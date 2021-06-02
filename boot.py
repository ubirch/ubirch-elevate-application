#!/usr/bin/env python
# Based on example OTA code from pycom which is from
# https://github.com/pycom/pycom-libraries/tree/master/examples/OTA
# Copyright notice for original example code:
#
# Copyright (c) 2019, Pycom Limited.
#
# This software is licensed under the GNU GPL version 3 or any
# later version, with permitted additional terms. For more information
# see the Pycom Licence v1.0 document supplied with this file, or
# available at https://www.pycom.io/opensource/licensing
#

import sys
import machine

#exit bootloader immediately if this is a deepsleep loop
if (machine.reset_cause() == machine.DEEPSLEEP_RESET):
    print("Deepsleep reset, skipping OTA bootloader")
    sys.exit()

import network
import math
import usocket as socket
import machine
import ujson
import uhashlib
import ubinascii
import gc
import pycom
import uos as os
import time
from time import sleep
import ubinascii
import ucrypto
from network import LTE
#only add compiled-in libraries above for reliability
#other code should be directly contained in boot.py

# To make the OTA bootloader self-contained, and hopefully more reliable, the library/classes are directly included here instead of an extra lib

#RSA PKCS1 PSS crypto functions
#This was ported to micropython based on the pycrypto library function in PKCS1_PSS.py
#See https://github.com/pycrypto/pycrypto/blob/7acba5f3a6ff10f1424c309d0d34d2b713233019/lib/Crypto/Signature/PKCS1_PSS.py
class PKCS1_PSSVerifier():
    def __init__(self,hasher=uhashlib.sha256):
        self.hasher = hasher

    def __get_hash(self,data):
        hasher = None
        try:
            hasher = self.hasher(data)
            return hasher.digest()
        except Exception as e:
            if hasher is not None:
                hasher.digest()#make sure hasher is closed, as only one is allowed at a time by the hardware
            raise e

    def __byte_xor(self,ba1, ba2):
        return bytes([_a ^ _b for _a, _b in zip(ba1, ba2)])

    def __modular_pow_public(self,message_int, exp_int, n_int):
        """Perform RSA function by exponentiation of message (= signature data) to exp with modulus n, all public.
        Only use with public parameters, as it will leak information about them. Micropython ints can be (almost) arbitrarily large.
        Based on micropython implementation from
        https://github.com/artem-smotrakov/esp32-weather-google-sheets/blob/841722fd67404588bfd29c15def897c9f8e967e3/src/rsa/common.py
        (Published under MIT License, Copyright (c) 2019 Artem Smotrakov)
        """

        if not isinstance(message_int,int) or not isinstance(exp_int,int)or not isinstance(n_int,int):
            raise Exception('Only integer inputs are supported')

        if message_int < 0:
            raise Exception('Only non-negative numbers are supported')

        if message_int >= n_int:
            raise Exception("The message %i is too long for n=%i" % (message_int, n_int))

        if n_int == 1:
            return 0
        # Assert :: (n - 1) * (n - 1) does not overflow message
        result = 1
        message_int = message_int % n_int
        while  exp_int > 0:
            if  exp_int % 2 == 1:
                result = (result * message_int) % n_int
            exp_int =  exp_int >> 1
            message_int = (message_int * message_int) % n_int
        return result

    def __EMSA_PSS_VERIFY(self,mhash, em, emBits, sLen):
        """
        Implement the ``EMSA-PSS-VERIFY`` function, as defined
        in PKCS#1 v2.1 (RFC3447, 9.1.2).
        ``EMSA-PSS-VERIFY`` actually accepts the message ``M`` as input,
        and hash it internally. Here, we expect that the message has already
        been hashed instead.
        :Parameters:
        mhash : Byte array that holds the digest of the message to be verified.
        em : bytes
                The signature to verify, therefore proving that the sender really signed
                the message that was received.
        emBits : int
                Length of the final encoding (em), in bits.
        sLen : int
                Length of the salt, in bytes.
        :Return: 0 if the encoding is consistent, 1 if it is inconsistent.
        :Raise ValueError:
            When digest or salt length are too big.
        """

        emLen = math.ceil(emBits/8)

        hLen = len(mhash)

        # Bitmask of digits that fill up
        lmask = 0
        for i in range(8*emLen-emBits):#was xrange before
            lmask = lmask>>1 | 0x80

        # Step 1 and 2 have been already done
        # Step 3
        if emLen < hLen+sLen+2:
            return False
        # Step 4
        if ord(em[-1:])!=0xBC:
            return False
        # Step 5
        maskedDB = em[:emLen-hLen-1]
        h = em[emLen-hLen-1:-1]
        # Step 6
        if lmask & em[0]:
            return False
        # Step 7
        dbMask = self.__MGF1(h, emLen-hLen-1)
        # Step 8
        db = self.__byte_xor(maskedDB, dbMask)
        # Step 9
        db = ((db[0]) & ~lmask).to_bytes(1,'big') + db[1:]
        # Step 10
        if not db.startswith((b'\x00')*(emLen-hLen-sLen-2) + (b'\x01')):
            return False
        # Step 11
        salt = b''
        if sLen: salt = db[-sLen:]
        # Step 12 and 13
        hp =self.__get_hash((b'\x00')*8 + mhash + salt)
        # Step 14
        if h!=hp:
            return False
        return True

    def __MGF1(self,mgfSeed, maskLen):
        """Mask Generation Function, described in B.2.1"""
        T = b''
        hashsize = len(self.__get_hash(""))
        for counter in range(math.ceil(maskLen/hashsize)):
            c = counter.to_bytes(4,'big')
            T = T + self.__get_hash(mgfSeed + c)
        assert(len(T)>=maskLen)
        return T[:maskLen]

    def changehashfunction(self,hasher):
        self.hasher = hasher

    def verify(self,mhash, signature_hex, pub_modulus_hex,pub_exponent = 65537,modBits = 2048):
            """Verify that a certain PKCS#1 PSS signature is authentic.

            This function checks if the party holding the private half of the given
            RSA key has really signed the message.

            This function is called ``RSASSA-PSS-VERIFY``, and is specified in section
            8.1.2 of RFC3447.

            :Parameters:
            mhash : bytes
                    The hash that was carried out over the message as raw bytes
            signature_hex : hex string
                    The signature that needs to be validated.
            pub_modulus_hex : hex string
                    The public modulus (pubkey main part) for verification.
            pub_exponent : integer
                    The public exponent (minor info/part of pubkey) for verification. Uses the common 65537 as default.
            modBits : integer
                    The bits the modulo n fits in. Genrally same as key length in bits, Defaults to 2048

            :Return: True if verification is correct. False otherwise.

            This was ported to micropython based on the pycrypto library function in PKCS1_PSS.py
            See https://github.com/pycrypto/pycrypto/blob/7acba5f3a6ff10f1424c309d0d34d2b713233019/lib/Crypto/Signature/PKCS1_PSS.py
            (Public domain, no rights reserved.)
            """

            #Verify input parameters

            #determine length of hash returned by currently set hash function
            hLen = len(self.__get_hash("0"))
            # salt length is assumed to be same as number of bytes in hash
            sLen = hLen

            #Check hash parameter has correct length
            if len(mhash) != hLen:
                print("Warning: Can't check signature, hash size is invalid.")
                return False

            #parse other parameters
            signature = ubinascii.unhexlify(signature_hex)
            pub_modulus = ubinascii.unhexlify(pub_modulus_hex)
            pub_modulus_int = int.from_bytes(pub_modulus, 'big')

            # See 8.1.2 in RFC3447
            k = math.ceil(modBits/8)
            # Step 1
            if len(signature) != k:
                print("Warning: Can't check signature, signature length wrong.")
                return False
            # Step 2a (O2SIP), 2b (RSAVP1), and partially 2c (I2OSP)
            # Note that signature must be smaller than the module
            # but we won't complain about it (here).
            sig_int = int.from_bytes(signature, 'big')
            m = self.__modular_pow_public(sig_int,pub_exponent, pub_modulus_int)
            # Step 2c (convert m int to em octet string)
            emLen = math.ceil((modBits-1)/8)
            em = m.to_bytes(emLen,'big')
            # Step 3
            try:
                result = self.__EMSA_PSS_VERIFY(mhash, em, modBits-1, sLen)
            except ValueError:
                return False
            # Step 4
            return result

class OTA():
    #set public key (modulus) of OTA server signature here:
    PUB_MOD_RSA_4096 = "A0A720B7520464C1DCFC71C80753AF9897C0B4F6A2066666BA5444CFE8BE5278EC08C838A2DBECD2B842B71909F0148DDF764933840F17194D5AB1DB06918E98D19441E6DDAD21F19BF520B480E95B0730E4DA77069EBB034BA269775980C41EE6BDFAD3C2AB98A7FBDC8B84F4359A7255C066307773D8A0E623A0F2A6F4C742364E613AB2342E6982B7CBB9B680EA6C8366A17529A75C6741423F0FDF7AE9C6C24E99FDB6A9CE431A10BD0604A01E2F59D8213590B5A3AFF012F73BB2D9C654FDA0CBE6D57CDA60E6CF433E680F0BA0C55C19D36223B92EFD4D66A2546AB5CA90EB1DD1781BF3F480A50B0CF23B7CE7CEE45938D4B0E6976EFE43EBB8E11DFCFA59DDF76E0475B0C921C7EDCE0B46B1A959B4314FD70E1D08067367D09F55154FA558C8A2A316F5372F8ED0EB0917677A480005C0CDCCE382F70FD7F2846BBFA0ACFFAB2103044CBB3488CD3A36C6CA1BE895B69DD3DF3E435E9375C692F22BC8445686D36EA8952BFB3B98B8E255F338CFEE961EFC23FBAFA314CC3CBC25C474FB7F75B680D20A31C67B56B759F4B452C2367331DF90900E78E3BE9052172DCF8224354B06994703CD5F28E707A6B4EDE584CB7019A388BE55F73C704D058CDCA7990E7E7330AADBE0EA52C0A2C2209964D6B8A9AC1092F9C94C2D2FF0F16EF9100C5D3F188481B44DD038BF03FA2BF9D719D25B5E663073F8F22C8EC71047"
    PROTOCOL_VERSION = "1.0" #version of the bootloader-server protocol

    # The following two methods need to be implemented in a subclass for the
    # specific transport mechanism e.g. WiFi

    def connect(self):
        raise NotImplementedError()

    def get_data(self, req, dest_path=None, hash=False, hasher=uhashlib.sha512):
        raise NotImplementedError()

    # The returned id string will be used in the request to the server to make device identification easier
    def get_device_id(self):
        raise NotImplementedError()

    # This method should be called when the update is done to close connections and deinit hardware etc.
    def clean_up(self):
        raise NotImplementedError()

    # OTA methods

    def get_current_version(self):
        try:
            from OTA_VERSION import VERSION
        except ImportError:
            VERSION = '1.0.0'
        return VERSION

    def check_manifest_signature(self,manifest:str,sig_type:str,sig_data:str)->bool:

        if(sig_type=="SIG01_PKCS1_PSS_4096_SHA256"):
            verifier = PKCS1_PSSVerifier(hasher=uhashlib.sha256)
            pub_modulus = self.PUB_MOD_RSA_4096
            hasher = uhashlib.sha256(manifest)
            manifest_hash = hasher.digest()
            return verifier.verify(manifest_hash,sig_data,pub_modulus,modBits=4096)
        elif(sig_type=="SIG02_PKCS1_PSS_4096_SHA512"):
            verifier = PKCS1_PSSVerifier(hasher=uhashlib.sha512)
            pub_modulus = self.PUB_MOD_RSA_4096
            hasher = uhashlib.sha512(manifest)
            manifest_hash = hasher.digest()
            return verifier.verify(manifest_hash,sig_data,pub_modulus,modBits=4096)
        else:
            raise Exception("Unknown signature type: {}".format(sig_type))

        raise Exception("Something is wrong with check_manifest_signature(), you should not have reached this line.")

    def extract_from_response(self,header:str,response:str)->(str,str):
        """Extracts the data from a server response using the header.
        Returns the data itself and the response with data and header removed
        Expected format: HEADER_NAME[len(data)]:data, e.g. MYDATA[3]:123SOMEOTHERDATA[2]:ab etc.
        """
        size_marker_start = "["
        size_marker_end = "]:"

        #find header including the begin of size field
        header_start = response.find(header+size_marker_start)
        if header_start == -1:#no valid header found
            return("",response)

        #determine start position of size integer
        size_start = header_start + len(header) + len(size_marker_start)

        #find end of size field
        size_end = response.find(size_marker_end, size_start)
        if size_end == -1:#end of size field not found
            return("",response)

        #extract size string and try conversion
        size_str = response[size_start:size_end]
        try:
            data_size = int(size_str)
        except:#size string conversion failed
            return("",response)

        #extract data string
        data_start = size_end+len(size_marker_end)
        data_end = data_start + data_size
        if data_end > len(response):#data size is larger than available data
            return("",response)
        data_str = response[data_start:data_end]

        #remove data and header from response
        remaining_response = response[:header_start]+response[data_end:]

        return(data_str,remaining_response)

    def get_update_manifest(self):
        """Get the manifest data from server and check signature.
        Gets the data, splits it into the strings for manifest (JSON), signature type, and signature data
        using the headers. Then checks the signature and finally parses the JSON of the manifest.
        """
        #generate request id string (based on random bits)
        request_id = ubinascii.hexlify(ucrypto.getrandbits(128)).decode('utf-8')
        req = "manifest.json?current_ver={}&devid={}&reqid={}&protocol={}".format(self.get_current_version(),self.get_device_id(),request_id,self.PROTOCOL_VERSION)
        response = self.get_data(req).decode()

        # print("response: {}".format(response))

        if len(response)==0 or response is None:
            raise Exception("No response received from server")

        # get the data from the headers and repeat with the remaining response
        # we get/remove the manifest json first as it is the most critical to remove
        # since it's data is arbitrary and might contain 'header-like' strings
        manifest_str,response = self.extract_from_response("MANIFEST",response)
        sig_type_str,response = self.extract_from_response("SIGNATURE_TYPE",response)
        sig_data_str,response = self.extract_from_response("SIGNATURE_DATA",response)

        # print("manifest:{}".format(manifest_str))
        # print("sig_type:{}".format(sig_type_str))
        # print("sig_data:{}".format(sig_data_str))

        #check that all data was found
        if len(manifest_str) == 0 or \
            len(sig_type_str) == 0 or \
            len(sig_data_str) == 0 :
            raise Exception("Could not find all required headers in response.")

        #check signature
        if self.check_manifest_signature(manifest_str,sig_type_str,sig_data_str):
            print("Signature OK, parsing manifest")
            manifest = ujson.loads(manifest_str)
        else:
            raise Exception("Signature of manifest is invalid")

        # check that this is the signature we requested and not a replay
        try:
            returned_req_id = manifest['request_id']
        except KeyError:
            raise Exception("Manifest invalid: no request ID returned")
        if returned_req_id != request_id:
            raise Exception("Manifest invalid: returned request ID does not match query request ID")

        gc.collect()
        return manifest

    def check_path(self, file_path):
        """ checks if the folder leaving to the given file path exist/creates them """
        # split the path into its parts
        # the last element is the file name; all before are folders
        path_parts = file_path.split("/")[:-1]

        # string to store the path of existent dirs
        current_path = "/"

        # go trough the parts
        for part in path_parts:
            # ignore potential empty parts originating from splitting up paths like
            #  /flash/... (leading /)
            #  /flash//lib (double /)
            if part.strip() == "":
                continue

            # assemble the new path
            new_path = current_path + part

            # check if the part (dir) is not found in the current directory
            # remove the trailing / if the path does consist of more than just the /
            # os.listdir("/flash/lib/") will cause an EINVAL error
            if not part in os.listdir(current_path[:-1] if (len(current_path) > 1 and current_path[-1] == "/") else current_path):
                # create it
                os.mkdir(new_path)

            # set the current_path and add a / for the next part
            current_path = new_path + "/"

    def update(self):
        manifest = self.get_update_manifest()

        #check if we are already on the latest version
        try:
            new_ver = manifest['new_version']
            old_ver = manifest['old_version']
        except KeyError:
            raise Exception("Manifest is invalid: could not parse old/new version information")
        if new_ver == old_ver:
            print("Already on the latest version")
            return

        #check if the manifest was generated for the correct version
        if old_ver != self.get_current_version():
            raise Exception("Manifest is invalid: Manifest is based on version different from the device version")

        # Download new files and verify hashes
        for f in manifest['new'] + manifest['update']:
            # check if the path exists/create it
            self.check_path(f['dst_path'])

            # Upto 5 retries
            for _ in range(5):
                try:
                    self.get_file(f)
                    break
                except Exception as e:
                    print(e)
                    msg = "Error downloading `{}` retrying..."
                    print(msg.format(f['URL']))
            else:
                raise Exception("Failed to download `{}`".format(f['URL']))

        # Rename new files to proper name
        for f in manifest['new'] + manifest['update']:
            new_path = "{}.new".format(f['dst_path'])
            dest_path = "{}".format(f['dst_path'])

            os.rename(new_path, dest_path)

        # delete files no longer required
        for f in manifest['delete']:
            self.delete_file(f)

        # Flash firmware
        if "firmware" in manifest:
            print("Warning: manifest contains board firmware update, but this is not implemented")
            # This is disabled as the current implementaion does not check the hash/buffers
            # the file and needs to be rewritten
            #self.write_firmware(manifest['firmware'])

        with open("/flash/OTA_VERSION.py", 'w') as fp:
            fp.write("VERSION = '{}'".format(manifest['new_version']))
        from OTA_VERSION import VERSION

        # Reboot the device to run the new code
        print("\nUpdate done. Resetting.")
        time.sleep(0.5)
        machine.reset()

    def get_file(self, f):
        new_path = "{}.new".format(f['dst_path'])

        # If a .new file exists from a previously failed update delete it
        try:
            os.remove(new_path)
        except OSError:
            pass  # The file didnt exist

        # Download new file with a .new extension to not overwrite the existing
        # file until the hash is verified.
        hash = self.get_data(f['URL'].split("/", 3)[-1],
                             dest_path=new_path,
                             hash=True)

        # Hash mismatch
        if hash != f['hash']:
            print(hash, f['hash'])
            msg = "Downloaded file's hash does not match expected hash"
            raise Exception(msg)

    def delete_file(self, f):
        dest_path = "/{}".format(f)

        try:
            os.remove(dest_path)
        except OSError as e:
           print("OTA: Cannot delete file %s: %s" % (dest_path, str(e)))

    # def write_firmware(self, f):
    #     hash = self.get_data(f['URL'].split("/", 3)[-1],
    #                          hash=True,
    #                          firmware=True)
    #     # TODO: Add verification when released in future firmware

    def check_stored_files(self) -> int:
        """ uses a hashlist to check all stored files for integrity """
        #
        # TODO checking/reloading boot.py is currently disabled
        #
        # the aim of the at-boot-time check is to detect compromised files and
        # reload them from the server to prevent unexpected runtime errors
        # it should only be run when no update was performed (firmware is up to date)
        # or the performed update was fully successful
        # if files were found to be corrupted and were replaced, the device should reboot
        # to prevent bootloops, this check failing should not be considered critical and
        # device should not reboot
        #
        # the function returns the number of reloaded files
        #
        # this function relies on the existence of a hashlist:
        #   /flash/hashlist.txt
        #
        # this file has the following layout: (to the left: line number)
        #   01 | HASH_ALGORITHM
        #   02 | RELATIVE_SERVER_RELEASE_PATH
        #   mn | HASH LOCAL_FILE_PATH
        #
        # a possible example would be:
        #   01 | md5
        #   02 | fw_release/1.0.1/
        #   03 | cc35b685ebaf792da25c006758890adf ./flash/lib/config.py
        #   04 | 4361596e9dade2612120d84a43455e57 ./flash/lib/urequests.py
        #   ...
        #
        # the hashlist must not contain itselfs
        #
        # supported hash algorithms currently are:
        #   md5, sha1, sha224, sha256, sha384 and sha512
        #
        # this limitation is based on the integrated mircropython library uhashlib
        # while not being considered secure for password hashing (...) md5 is the
        # favorable choice in this scenario because it is significantly lighter both
        # in storage and computing compared to sha256 or sha512
        # also note that the function losely matches HASH_ALGORITHM so that sha1sum will
        # be interpreted as sha1; this also means that some imaginary - unsupported -
        # algorithm like Bsha2561 will be interpreted as sha256
        #
        # the LOCAL_FILE_PATH is allowed to ether beging with a '/' or a './'
        # for the latter the client (here) needs to remove the '.' before using the
        # path since Micropythons implementation of os.chdir() does not move to '/flash'
        # when called from '/' with os.chdir("./flash")
        #

        reloaded_n = 0

        print("############ Beginning integrity check of stored files ############")

        # check for "/flash/hashlist.txt"
        try:
            hl_fd = open("/flash/hashlist.txt", "r")
        except OSError:
            print(
                "[*] Aborting integrity check, hashlist not found (/flash/hashlist.txt)")
            print("###################################################################")

            return reloaded_n

        # get the hash algorithm and initialise the hasher
        hash_alg_str = hl_fd.readline().strip("\n")

        if "md5" in hash_alg_str:
            _hasher = uhashlib.md5
        elif "sha1" in hash_alg_str:
            _hasher = uhashlib.sha1
        elif "sha224" in hash_alg_str:
            _hasher = uhashlib.sha224
        elif "sha256" in hash_alg_str:
            _hasher = uhashlib.sha256
        elif "sha384" in hash_alg_str:
            _hasher = uhashlib.sha384
        elif "sha512" in hash_alg_str:
            _hasher = uhashlib.sha512
        else:
            print(
                "[!] Aborting integrity check, unsupported hash algorithm: %s!" % hash_alg_str)
            print("###################################################################")

            return reloaded_n

        print("[*] Using %s hashing for integrity check" % _hasher.__name__)

        # get the relative server-side release path
        rel_release_path = hl_fd.readline().strip("\n")

        print("[*] Relative server side release path set to %s" %
              rel_release_path)

        # go trough the actual hash list
        for _line in hl_fd.readlines():
            line = _line.strip("\n").split()

            # check if there are exactly two elements; hash and path
            if len(line) != 2:
                print("[!] Formatting errors in the hashlist!")

                break

            hash_str = line[0]
            path_str = line[1]

            # skip if the path matches boot.py
            # TODO this might be changed later
            if path_str.split("/")[-1] == "boot.py":
                continue

            # bring the path into a usable format
            if path_str[0] == ".":
                path_str = path_str[1:]

            path_tmp = path_str + ".tmp"  # path for the download

            print("[*] Checking %s:%s" % (path_str, hash_str))

            # try to open the file
            try:
                f_fd = open(path_str, "rb")
            except OSError as e:
                print("[!] Error opening file %s: %s" % (path_str, str(e)))
            else:
                # create the hasher
                hasher = _hasher()

                # feed the file into the hasher
                hasher.update(f_fd.read())

                # close the file
                f_fd.close()

                # get the hash
                hashv = ubinascii.hexlify(hasher.digest()).decode()

                # compare the hash values
                if hashv != hash_str:
                    print("[M] Mismatching hashes for %s: %s (is) vs. %s (should be)" % (
                        path_str, hashv, hash_str))
                else:
                    # file was found and the hash is correct; go to the next one
                    continue

            # file not found or hash incorrect -> reload the file; try 5 imes
            http_get_path = "%s/%s" % (rel_release_path, path_str)

            print("[*] Trying to load %s from the server to %s" %
                  (http_get_path, path_tmp))

            failed = False

            for i in range(5):
                try:
                    # save the file to FILENAME.tmp
                    dl_hash = self.get_data(
                        http_get_path, path_tmp, hash=True, hasher=_hasher)
                except Exception as e:
                    print("[!] %s" % str(e))

                    if i != 4:
                        print("[!] Error getting %s! Trying again %d more times" % (
                            http_get_path, 5 - i))
                    else:
                        print("[!] Finally failed to load %s!" % http_get_path)

                        failed = True
                else:
                    break

            if failed == True:
                # failure - abort
                break

            # check if the hash of the downloaded file matches
            if dl_hash != hash_str:
                print("[M] Downloaded file hash mismatch %s: %s (is) vs. %s (should be)" % (
                    http_get_path, dl_hash, hash_str))

                break

            # the hash is correct -> file is good; replace the old one
            try:
                os.remove(path_str)
            except:
                # remove the file is expected to fail is the fail for some reason did not exist
                pass

            os.rename(path_tmp, path_str)

            # successfully reloaded a file
            reloaded_n += 1

            print("[*] Successfully reloaded %s from the OTA server" % path_str)

        hl_fd.close()

        print("###################################################################")

        return reloaded_n


class WiFiOTA(OTA):
    def __init__(self, ssid, password,sockettimeout:int=60):
        self.SSID = ssid
        self.password = password
        self.sockettimeout = sockettimeout
        self.ip = None #IP is either set or resolved by "connect" function
        self.port = None #Port is  set by "connect" function


    def get_device_id(self):
        #TODO For Wifi, change this to something like the mac address instead of SIM ICCID
        """Get an identifier for the device used in server requests
        In this case, we return the SHA256 hash of the SIM ICCID prefixed with
        'ID:' and then prepend an 'IC' (for ICCID) so the result is e.g.
        devid = 'IC' + SHA256("ID:12345678901234567890")
              = IC27c6bb74efe9633181ae95bade7740969df13ef15bca1d72a92aa19fb66d24c9"""

        try:
            lte = LTE()
            iccid = lte.iccid()
        except Exception as e:
            print("Getting device ID failed:")
            print(e)
            return "ICERROR"

        hasher = None
        try:
            hasher = uhashlib.sha256("ID:"+iccid)
            hashvalue = hasher.digest()
        except Exception as e:
            if hasher is not None:
                hasher.digest()#make sure hasher is closed, as only one is allowed at a time by the hardware
            raise e

        devid = "IC" + ubinascii.hexlify(hashvalue).decode('utf-8')

        return devid

    def connect(self, port:int,url:str="",ip:str=""):
        """
        Connects to the transport network, sets server port, and resolves update server IP from URL if necessary. Specify either IP or URL, not both.
        """
        self.wlan = network.WLAN(mode=network.WLAN.STA)
        if not self.wlan.isconnected() or self.wlan.ssid() != self.SSID:
            for net in self.wlan.scan():
                if net.ssid == self.SSID:
                    self.wlan.connect(self.SSID, auth=(network.WLAN.WPA2,
                                                       self.password))
                    while not self.wlan.isconnected():
                        machine.idle()  # save power while waiting
                    break # at this point, we are connected, leave for loop
            else:
                raise Exception("Cannot find network '{}'".format(self.SSID))

        print("Connected to SSID {}".format(self.wlan.ssid()))

        #set port
        self.port = port

        #check parameters and handle resolve/ip setting accordingly
        if (url == "" and ip == "") or (url != "" and ip != ""): # nothing given or both given
            raise OSError("No URL or IP, or both at the same time given")
        elif url != "": # url given
            if ":" in url:
                raise OSError("Please specify port as separate parameter and not in URL")
            print("Resolving server URL {}...".format(url))
            try:
                ai = socket.getaddrinfo(url, port)
                self.ip = ai[0][-1][0]
            except Exception as e:
                raise OSError("IP address could not be resolved: {}".format(e))
        elif ip != "": # ip given
            self.ip = ip
        print("Update server address: {}:{}".format(self.ip,self.port))


    def clean_up(self):
        if self.wlan.isconnected():
            self.wlan.disconnect()
        self.wlan.deinit()

    def _http_get(self, path, host):
        req_fmt = 'GET /{} HTTP/1.0\r\nHost: {}\r\n\r\n'
        req = bytes(req_fmt.format(path, host), 'utf8')
        return req

    def get_data(self, req, dest_path=None, hash=False, hasher=uhashlib.sha512):
        # board firmware download is disabled as it needs to
        # be rewritten to not ignore the file hash
        firmware=False

        h = None

        # Connect to server
        print("Requesting: {}".format(req))
        s = socket.socket(socket.AF_INET,
                          socket.SOCK_STREAM,
                          socket.IPPROTO_TCP)
        s.settimeout(self.sockettimeout)
        s.connect((self.ip, self.port))

        # Request File
        s.sendall(self._http_get(req, "{}:{}".format(self.ip, self.port)))

        try:
            content = bytearray()
            fp = None
            if dest_path is not None:
                if firmware:
                    raise Exception("Cannot write firmware to a file")
                fp = open(dest_path, 'wb')

            if firmware:
                pycom.ota_start()

            h = hasher()

            # Get data from server
            result = s.recv(100)

            start_writing = False
            while (len(result) > 0):
                # Ignore the HTTP headers
                if not start_writing:
                    if "\r\n\r\n" in result:
                        start_writing = True
                        result = result.decode().split("\r\n\r\n")[1].encode()

                if start_writing:
                    if firmware:
                        pycom.ota_write(result)
                    elif fp is None:
                        content.extend(result)
                    else:
                        fp.write(result)

                    if hash:
                        h.update(result)

                result = s.recv(100)

            s.close()

            if fp is not None:
                fp.close()
            if firmware:
                pycom.ota_finish()

        except Exception as e:
            # Since only one hash operation is allowed at Once
            # ensure we close it if there is an error
            if h is not None:
                h.digest()
            raise e

        hash_val = ubinascii.hexlify(h.digest()).decode()

        if dest_path is None:
            if hash:
                return (bytes(content), hash_val)
            else:
                return bytes(content)
        elif hash:
            return hash_val

class NBIoTOTA(OTA):
    def __init__(self, lte: LTE, apn: str, band: int or None, attachtimeout: int, connecttimeout:int,sockettimeout:int = 60):
        self.lte = lte
        self.apn = apn
        self.band = band
        self.attachtimeout = attachtimeout
        self.connecttimeout = connecttimeout
        self.sockettimeout = sockettimeout
        self.ip = None #IP is either set or resolved by "connect" function
        self.port = None #Port is  set by "connect" function

    def attach(self):
        if self.lte.isattached():
            return

        sys.stdout.write("Attaching to the NB-IoT network")
        # since we disable unsolicited CEREG messages, as they interfere with AT communication with the SIM via CSIM commands,
        # we are required to use an attach method that does not require cereg messages, for pycom that is legacyattach=false
        self.lte.attach(band=self.band, apn=self.apn,legacyattach=False)
        i = 0
        while not self.lte.isattached() and i < self.attachtimeout:
            i += 1
            time.sleep(1.0)
            sys.stdout.write(".")
        if not self.lte.isattached():
            raise OSError("Timeout when attaching to NB-IoT network.")

        print("\nattached: {} s".format(i))

    def connect(self, port:int,url:str="",ip:str=""):
        """
        Connects to the transport network, sets server port, and resolves update server IP from URL if necessary. Specify either IP or URL, not both.
        """
        if not self.lte.isattached():
            self.attach()

        if not self.lte.isconnected():
            sys.stdout.write("Connecting to the NB-IoT network")
            self.lte.connect()  # start a data session and obtain an IP address
            i = 0
            while not self.lte.isconnected() and i < self.connecttimeout:
                i += 1
                time.sleep(1.0)
                sys.stdout.write(".")
            if not self.lte.isconnected():
                raise OSError("Timeout when connecting to NB-IoT network.")

        print("\nconnected in {} s".format(i))

        #set port
        self.port = port

        #check parameters and handle resolve/ip setting accordingly
        if (url == "" and ip == "") or (url != "" and ip != ""): # nothing given or both given
            raise OSError("No URL or IP, or both at the same time given")
        elif url != "": # url given
            if ":" in url:
                raise OSError("Please specify port as separate parameter and not in URL")
            print("Resolving server URL {}...".format(url))
            #set dns servers
            socket.dnsserver(1, '8.8.4.4')
            socket.dnsserver(0, '8.8.8.8')
            try:
                ai = socket.getaddrinfo(url, port)  # todo check if -> is needed, 0, usocket.SOCK_STREAM)
                self.ip = ai[0][-1][0]
            except Exception as e:
                raise OSError("IP address could not be resolved: {}".format(e))
        elif ip != "": # ip given
            self.ip = ip
        print("Update server address: {}:{}".format(self.ip,self.port))



    def clean_up(self):
        if self.lte.isconnected():
            self.lte.disconnect()
        self.lte.deinit()

    def get_device_id(self):
        """Get an identifier for the device used in server requests
        In this case, we return the SHA256 hash of the SIM ICCID prefixed with
        'ID:' and then prepend an 'IC' (for ICCID) so the result is e.g.
        devid = 'IC' + SHA256("ID:12345678901234567890")
              = IC27c6bb74efe9633181ae95bade7740969df13ef15bca1d72a92aa19fb66d24c9"""

        try:
            connection = self.lte.isconnected() #save connection state on entry
            if connection:#we are connected at this point and must pause the data session
                self.lte.pppsuspend()
            iccid = self.lte.iccid()
            if connection:#if there was a connection, resume it
                self.lte.pppresume()
        except Exception as e:
            print("Getting device ID failed:")
            print(e)
            return "ICERROR"

        hasher = None
        try:
            hasher = uhashlib.sha256("ID:"+iccid)
            hashvalue = hasher.digest()
        except Exception as e:
            if hasher is not None:
                hasher.digest()#make sure hasher is closed, as only one is allowed at a time by the hardware
            raise e

        devid = "IC" + ubinascii.hexlify(hashvalue).decode('utf-8')

        return devid

    def _http_get(self, path, host):
        req_fmt = 'GET /{} HTTP/1.0\r\nHost: {}\r\n\r\n'
        req = bytes(req_fmt.format(path, host), 'utf8')
        return req

    def get_data(self, req, dest_path=None, hash=False, hasher=uhashlib.sha512):
        # board firmware download is disabled as it needs to
        # be rewritten to not ignore the file hash
        firmware=False

        h = None

        # Connect to server
        print("Requesting: {}".format(req))
        s = socket.socket(socket.AF_INET,
                          socket.SOCK_STREAM,
                          socket.IPPROTO_TCP)
        s.settimeout(self.sockettimeout)
        s.connect((self.ip, self.port))

        # Request File
        s.sendall(self._http_get(req, "ota.iot.wheelmap.pro")) # "{}:{}".format(self.ip, self.port)))

        try:
            content = bytearray()
            fp = None
            if dest_path is not None:
                if firmware:
                    raise Exception("Cannot write firmware to a file")
                fp = open(dest_path, 'wb')

            if firmware:
                pycom.ota_start()

            h = hasher()

            # Get data from server
            result = s.recv(100)

            start_writing = False
            while (len(result) > 0):
                # Ignore the HTTP headers
                if not start_writing:
                    if "\r\n\r\n" in result:
                        start_writing = True
                        result = result.decode().split("\r\n\r\n")[1].encode()

                if start_writing:
                    if firmware:
                        pycom.ota_write(result)
                    elif fp is None:
                        content.extend(result)
                    else:
                        fp.write(result)

                    if hash:
                        h.update(result)

                result = s.recv(100)

            s.close()

            if fp is not None:
                fp.close()
            if firmware:
                pycom.ota_finish()

        except Exception as e:
            # Since only one hash operation is allowed at Once
            # ensure we close it if there is an error
            if h is not None:
                h.digest()
            raise e

        hash_val = ubinascii.hexlify(h.digest()).decode()

        if dest_path is None:
            if hash:
                return (bytes(content), hash_val)
            else:
                return bytes(content)
        elif hash:
            return hash_val


# helper function to perform the update and keep the OTA
# objects out of global scope (boot.py and main.py have the same scope)
# and thus allow for garbage collection of OTA memory usage later
def check_OTA_update():
    # Configuration (if you are looking for the server pubkey: it's in the OTA class)
    #TODO: Change server URL and Pubkey to non-testing versions
    SERVER_URL = "ota.iot.wheelmap.pro"
    SERVER_PORT = 80
    NBIOT_APN = "iot.1nce.net"
    NBIOT_BAND = None #None = autoscan
    NBIOT_ATTACH_TIMEOUT = 15*60 #seconds
    NBIOT_CONNECT_TIMEOUT = 15*60 #seconds
    WATCHDOG_TIMEOUT =  15*60*1000 #milliseconds

    #setup watchdog
    wdt = machine.WDT(timeout=WATCHDOG_TIMEOUT)
    wdt.feed()

    #initialize ota object variable for proper exception handling
    ota = None

    try:
        # # Setup Wifi OTA
        # from ota_wifi_secrets import WIFI_SSID, WIFI_PW
        # ota = WiFiOTA(WIFI_SSID,
        #         WIFI_PW)

        # Setup NB-IoT OTA
        print("Initializing LTE")
        lte = LTE()
        lte.reset()
        lte.init()

        ota = NBIoTOTA(lte,
                NBIOT_APN,
                NBIOT_BAND,
                NBIOT_ATTACH_TIMEOUT,
                NBIOT_CONNECT_TIMEOUT)

        #start the update itself
        print("Current version: ", ota.get_current_version())
        ota.connect(url=SERVER_URL,port=SERVER_PORT)
        ota.update()

        # the update did not fail, proceed to check file integrity
        n = ota.check_stored_files()

        if n > 0:
            print("%d corrupted files detected and reloaded! Rebooting ..." % n)

            time.sleep(0.5)
            machine.reset()
    except Exception as e:
        raise(e)#let top level loop handle exception
    finally:
        if ota is not None:
            ota.clean_up()
        # before leaving, set watchdog to large value, so we don't interfere
        # with code in main.py (wdt can never be disabled after use)
        wdt = machine.WDT(timeout=10*24*60*60*1000)
        wdt.feed()

### start main code ###

# Turn on GREEN LED
print("\nEntering OTA bootloader")
pycom.heartbeat(True)
# pycom.rgbled(0x000500)

# disable the FTP Server
server = network.Server()
server.deinit() # disable the server
# disable the wifi on boot
pycom.wifi_on_boot(False)

ota_max_retries = 2
for retries_left in range((ota_max_retries-1),-1,-1):
    try:
        print("\nStarting OTA update")
        check_OTA_update()
        break # leave retry for loop if successful
    except Exception as e:
        print("Exception occured during OTA update:")
        sys.print_exception(e)
        if retries_left > 0:
            print("Retrying update")
        else:
            print("Giving up")

gc.collect() #free up memory that was used by OTA objects
print("\nLeaving OTA bootloader")
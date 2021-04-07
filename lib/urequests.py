"""
:origin: https://github.com/pfalcon/pycopy-lib/tree/master/urequests
"""
import usocket

SOCKET_TIMEOUT_S = 120

class Response:

    def __init__(self, f):
        self.raw = f
        self.encoding = "utf-8"
        self._cached = None

    def close(self):
        if self.raw:
            self.raw.close()
            self.raw = None
        self._cached = None

    @property
    def content(self):
        if self._cached is None:
            try:
                self._cached = self.raw.read()
            finally:
                self.raw.close()
                self.raw = None
        return self._cached

    @property
    def text(self):
        return str(self.content, self.encoding)

    def json(self):
        import ujson
        return ujson.loads(self.content)

# storage for ssl sessions, for faster connection
ssl_stored_sessions = dict({})

def request(method, url, data=None, json=None, headers=None, stream=None, parse_headers=True):
    if headers is None:
        headers = {}
    global ssl_stored_sessions
    redir_cnt = 1
    if json is not None:
        assert data is None
        import ujson
        data = ujson.dumps(json)

    while True:
        try:
            proto, dummy, host, path = url.split("/", 3)
        except ValueError:
            proto, dummy, host = url.split("/", 2)
            path = ""
        if proto == "http:":
            port = 80
        elif proto == "https:":
            import ussl
            port = 443
        else:
            raise ValueError("Unsupported protocol: " + proto)

        if ":" in host:
            host, port = host.split(":", 1)
            port = int(port)

        usocket.dnsserver(1, '8.8.4.4')
        usocket.dnsserver(0, '8.8.8.8')
        ai = usocket.getaddrinfo(host, port, 0, usocket.SOCK_STREAM)
        ai = ai[0]

        resp_d = None
        if parse_headers is not False:
            resp_d = {}

        s = usocket.socket(ai[0], ai[1], ai[2])
        s.settimeout(SOCKET_TIMEOUT_S)
        try:
            if proto == "https:":
                if host in ssl_stored_sessions:
                    print("##### found session")
                    session = ssl_stored_sessions[host]
                    try:
                        s = ussl.wrap_socket(s, server_hostname=host, saved_session=session)
                        s.connect(ai[-1])
                        ssl_stored_sessions.pop(host)
                    except Exception as e:
                        print("##### error with saved session {}".format(repr(e)))
                        return
                else:
                    print("##### no session found")
                    s = ussl.wrap_socket(s, server_hostname=host)
                    s.connect(ai[-1])

                ssl_stored_sessions.update({host: ussl.save_session(s)})
                if len(ssl_stored_sessions) > 16:
                    print("##### too many stored session, will be cleared")
                    ssl_stored_sessions.clear()
            else:   # if HTTP only
                s.connect(ai[-1])
            s.write(b"%s /%s HTTP/1.0\r\n" % (method, path))
            if not "Host" in headers:
                s.write(b"Host: %s\r\n" % host)
            # Iterate over keys to avoid tuple alloc
            for k in headers:
                s.write(k)
                s.write(b": ")
                s.write(headers[k])
                s.write(b"\r\n")
            if json is not None:
                s.write(b"Content-Type: application/json\r\n")
            if data:
                s.write(b"Content-Length: %d\r\n" % len(data))
            s.write(b"Connection: shutdown\r\n\r\n")
            if data:
                s.write(data[:int(len(data)/2)])
                s.write(data[int(len(data)/2):])

            l = s.readline()
            #print(l)
            l = l.split(None, 2)
            status = int(l[1])
            reason = ""
            if len(l) > 2:
                reason = l[2].rstrip()
            while True:
                l = s.readline()
                if not l or l == b"\r\n":
                    break
                #print(l)

                if l.startswith(b"Transfer-Encoding:"):
                    if b"chunked" in l:
                        raise ValueError("Unsupported " + l.decode())
                elif l.startswith(b"Location:") and 300 <= status <= 399:
                    if not redir_cnt:
                        raise ValueError("Too many redirects")
                    redir_cnt -= 1
                    url = l[9:].decode().strip()
                    #print("redir to:", url)
                    status = 300
                    break

                if parse_headers is False:
                    pass
                elif parse_headers is True:
                    l = l.decode()
                    k, v = l.split(":", 1)
                    resp_d[k] = v.strip()
        except OSError:
            s.close()
            raise

        if status != 300:
            break

    resp = Response(s)
    resp.status_code = status
    resp.reason = reason
    if resp_d is not None:
        resp.headers = resp_d
    return resp


def head(url, **kw):
    return request("HEAD", url, **kw)

def get(url, **kw):
    return request("GET", url, **kw)

def post(url, **kw):
    return request("POST", url, **kw)

def put(url, **kw):
    return request("PUT", url, **kw)

def patch(url, **kw):
    return request("PATCH", url, **kw)

def delete(url, **kw):
    return request("DELETE", url, **kw)

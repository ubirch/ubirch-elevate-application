# Todos for the next release

## PIN broken

* If pin is not valid
  * disable ubirching
  * send error to wheelmap
  * delete <imsi>.bin
  * adapt exception handling at [init_sim_proto](system.py#L155)
  

## init lte modem 

* will stay like this for now

## load config

* already done

## [load_sim_pin](system.py#L108)

* try except for ensure.connection
* try except for bootstrap
* forward exception

## [init_sim_proto](system.py#L127)

* try except for [ElevateSim](system.py#L130)
* try except for [get_uuid](system.py#159)

## [get_csr](system.py#L162)

* can be removed and also everything related

## [send_event](system.py#L201)

* only `return` without bool
* throw exception [here](system.py#L301)

# [send_emergency_event](system.py#L315)

* like above with descriptive exception

# [get_state_from_backend](system.py#L342)

* replace `return None, None` by comment




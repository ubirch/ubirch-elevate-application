# Todos for the next release

## Create System class

It would be good to separate the `StateMachine` into `StateMachine` and 
`System`. 

`System` should be responsible for:
* [send_event()](./state_machine.py#L137)
* [send_emergency_event()](./state_machine.py#L243)
* [get_state_from_backend()](./state_machine.py#L269)
* init_system() (not yet defined) The code base is in 
  [StateInitSystem._update()](./state_machine.py#L433)

### move exception handling one level up

The exception handling currently deals with the exceptions itself. 
This should be moved one level higher, by `raise Exception`.

### remove StateMachine commands from System

Commands like `machine.go_to_state()` should only be called in the
`StateMachine`, not the `System` class.


### Example

```python
def send_something():
    error = False
    # initialise ubirch SIM protocol
    log.info("Initializing ubirch SIM protocol")
    try:
        machine.sim = ElevateSim(lte=machine.lte, at_debug=machine.debug)
    except Exception as e:
        machine.lastError = str(e)
        error = True #   machine.go_to_state('error')    
    return error 

def main():
    while(True):
    error = send_something()
    if error:
        go_to_state('error')
    else:
        pass # just continue with the rest of the code
```

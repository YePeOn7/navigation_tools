# from pynput import keyboard
# import time

# def on_press(key):
#     print(key)
#     # try:
#     #     print('alphanumeric key {0} pressed'.format(key.char))
#     # except AttributeError:
#     #     print('special key {0} pressed'.format(key))

# def on_release(key):
#     pass
#     # print('{0} released'.format(key))
#     # if key == keyboard.Key.esc:
#     #     # Stop listener
#     #     return False

# # listener = keyboard.Listener(on_press=on_press, on_release=on_release)
# # listener.start()
# with keyboard.Listener(
#         on_press=on_press,
#         on_release=on_release) as listener:
#     listener.join()

# while 1:  
#     print('a')
#     time.sleep(0.1)    

from pynput import keyboard

# The event listener will be running in this block
while 1:
    with keyboard.Events() as events:
        # Block at most one second
        event = events.get(1.0)
        if event is None:
            print('You did not press a key within one second')
        else:
            print('Received event {}'.format(event))
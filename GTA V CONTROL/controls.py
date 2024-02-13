from directkeys import PressKey,ReleaseKey, W, A, S, D
from time import sleep
def straight():
    PressKey(W)
    #sleep(1)
    #ReleaseKey(W)
    ReleaseKey(A)
    ReleaseKey(D)
    

def left():
    #PressKey(W)
    PressKey(A)
    ReleaseKey(D)
    #ReleaseKey(W)
    #sleep(0.01)
    ReleaseKey(A)
    

def right():
    #PressKey(W)
    PressKey(D)
    ReleaseKey(A)
    #ReleaseKey(W)
    #sleep(0.01)
    ReleaseKey(D)
    

def slow():
    ReleaseKey(W)
    ReleaseKey(A)
    ReleaseKey(D)
    #sleep(1)
    '''
    PressKey(S)
    sleep(0.1)
    ReleaseKey(S)
    ReleaseKey(W)
    ReleaseKey(A)
    ReleaseKey(D)
    '''
    #PressKey(W)
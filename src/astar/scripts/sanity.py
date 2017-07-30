from math import floor

def rmo(x, y, len):
    return x*len+y

def rmo2point(idx, len):
    x = floor(idx / len)
    y = idx - (x*len)
    return (int(x),int(y))


for x in range(10):
    for y in range(10):
        idx = rmo(x, y, 10)
        aX, aY = rmo2point(idx,10)
        if x != aX and y != aY:
            print("failed")
            print((x,y))
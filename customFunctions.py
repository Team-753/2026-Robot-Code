from math import pow,atan2,sqrt,sin,cos
def clamp(input,min,max):
    if input>max:
        input=max
    elif input<min:
        input=min
    return input
def curveControl(input,exponet):
    invert=1
    if input<0:
        input=abs(input)
        invert=-1
    input=clamp(input,0,1)
    variable=pow(input,exponet)*invert
    return variable
def vectorCurve(x,y,exponet,mult):
    angle=atan2(y,x)
    linearVelocity=sqrt((pow(x,2)+pow(y,2)))
    curvedVelocity=curveControl(linearVelocity,exponet)
    return[(sin(angle)*curvedVelocity*mult),(cos(angle)*curvedVelocity*mult)]
def thresholdEqual(value,desiredValue,buffer):
    if (desiredValue-buffer)<=value<=(desiredValue+buffer):
        return True
    return False
def estimate(array,point):
    for i in range(len(array)):
        if i != len(array):
            if array[i][0]<point and array[i+1][0]>point:
                dX=array[i+1][0]-array[i][0]
                dY=array[i+1][1]-array[i][1]
                slope=dY/dX
                dX2=point-array[i][0]
                dY2=dX2*slope
                output=dY2+array[i][1]
                return output
    return 0
def pythag(x1,y1,x2,y2):
    return sqrt(((x1-x2)**2)+((y1-y2)**2))
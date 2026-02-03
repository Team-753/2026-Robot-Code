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
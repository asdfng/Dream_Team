import json, urllib.request, time

while 1:

    with urllib.request.urlopen('http://172.16.0.1:8001/FieldData/GetData') as response:
        source = response.read()

    data = json.loads(source.decode())
    
    #Set up the Blue team Stats
    
    Blue = data['Blue Team Data']

    #Blue Circle
    bCircle = Blue['Circle']
    bcCenterPoint = bCircle['Object Center']
    bccpX = bcCenterPoint['X']
    bccpY = bcCenterPoint['Y']

    bcBoundinBox = bCircle['Bounding Box']
    bcbbXL = bcBoundinBox['X Left']
    bcbbYT = bcBoundinBox['Y Top']
    bcbbXR = bcBoundinBox['X Right']
    bcbbYB = bcBoundinBox['Y Bottom']

    bcArea = bCircle['Area']

    #Blue Square
    
    

    print(json.dumps(bccpX, indent=2))
    time.sleep(2)
    

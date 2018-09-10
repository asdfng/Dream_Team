import json, urllib.request, time

while 1:

    with urllib.request.urlopen('http://172.16.0.1:8001/FieldData/GetData') as response:
        source = response.read()

    data = json.loads(source.decode())

#Ball data parsing
    
    ball = data['Ball']
    ball_coordinates = ball['Object Center']
    ball_coordinateX = ball_coordinates['X']
    ball_coordinateY = ball_coordinates['Y']

    #Write ball data to ROS message (.msg)

    f = open( 'ball_coordinates.msg', 'w' )
    f.write( 'ball_x = ' + str(ball_coordinateX) + '\n' + 'ball_y = ' +  str(ball_coordinateY) + '\n')
    f.close()

#Corners data parsing

    corners = data['Corners']
    cornerBL = corners[0]
    cornerBR = corners[1]
    cornerTL = corners[2]
    cornerTR = corners[3]
    cornerBLX = cornerBL['X']
    cornerBLY = cornerBL['Y']
    cornerBRX = cornerBR['X']
    cornerBRY = cornerBR['Y']
    cornerTLX = cornerTL['X']
    cornerTLY = cornerTL['Y']
    cornerTRX = cornerTR['X']
    cornerTRY = cornerTR['Y']

    #Write corner data to ROS message (.msg)

    g = open( 'corners_coordinates.msg', 'w' )
    g.write('corners_BLX = ' + str(cornerBLX) + '\n' + 'corners_BLY = ' +  str(cornerBLY) + '\n' + 'corners_TLX = ' + str(cornerTLX) + '\n' + 'corners_TLY = ' +  str(cornerTLY) + '\n' \
            'corners_BRX = ' + str(cornerBRX) + '\n' + 'corners_BRY = ' +  str(cornerBRY) + '\n' + 'corners_TRX = ' + str(cornerTRX) + '\n' + 'corners_TRY = ' +  str(cornerTRY) + '\n')
    g.close
    
#Red Team data parsing
    
    Red = data['Red Team Data']

#Red Circle
    
    rCircle = Red['Circle']
    rcCenterPoint = rCircle['Object Center']
    
    #Coordinates
    
    rccpX = rcCenterPoint['X']
    rccpY = rcCenterPoint['Y']
    
    #Bounding Box
    
    rcBoundinBox = rCircle['Bounding Box']
    rcbbXL = rcBoundinBox['X Left']
    rcbbYT = rcBoundinBox['Y Top']
    rcbbXR = rcBoundinBox['X Right']
    rcbbYB = rcBoundinBox['Y Bottom']
    
    #Area
    
    rcArea = rCircle['Area']
    
#Red Square
    
    rSquare = Red['Square']
    rsCenterPoint = rSquare['Object Center']

    #Coordinates
    
    rscpX = rsCenterPoint['X']
    rscpY = rsCenterPoint['Y']
    
    #Bounding Box
    
    rsBoundinBox = rSquare['Bounding Box']
    rsbbXL = rsBoundinBox['X Left']
    rsbbYT = rsBoundinBox['Y Top']
    rsbbXR = rsBoundinBox['X Right']
    rsbbYB = rsBoundinBox['Y Bottom']
    
    #Area
    
    rsArea = rSquare['Area']
    
#Red Triangle
    
    rTriangle = Red['Triangle']
    rtCenterPoint = rTriangle['Object Center']
    
    #Coordinates
    
    rtcpX = rtCenterPoint['X']
    rtcpY = rtCenterPoint['Y']
    
    #Bounding Btox
    
    rtBoundinBox = rTriangle['Bounding Box']
    rtbbXL = rtBoundinBox['X Left']
    rtbbYT = rtBoundinBox['Y Top']
    rtbbXR = rtBoundinBox['X Right']
    rtbbYB = rtBoundinBox['Y Bottom']
    
    #Area
    
    rtArea = rTriangle['Area']

    #Write Red team data to ROS message (.msg)
    
    h = open( 'red_team_coordinates.msg', 'w' )
    h.write( 'circleX ' + str(rccpX) + '\n' + 'circleY = ' +  str(rccpY) + '\n' + 'circleBBXL ' + str(rcbbXL) + '\n' + 'circleBBXR = ' +  str(rcbbXR) + '\n' + 'circleBBYT ' + str(rcbbYT) + '\n' \
    + 'circleBBYB = ' +  str(rcbbYB) + '\n' + 'CircleArea = ' +  str(rcArea) + '\n' +'squareX = ' + str(rscpX) + '\n' + 'squareY = ' +  str(rscpY) + '\n' + 'squareBBXL = ' + str(rsbbXL) + \
    '\n' + 'squareBBXR = ' +  str(rsbbXR) + '\n' + 'squareYT ' + str(rsbbYT) + '\n' + 'squareYB = ' +  str(rsbbYB) + '\n' +  '/n' + 'squareArea = ' +  str(rsArea) + '/n' \
    'triangleX ' + str(rtcpX) + '\n' + 'triangleY = ' +  str(rtcpY) + '\n' + 'triangleBBXL ' + str(rtbbXL) + '\n' + 'triangleBBXR = ' +  str(rtbbXR) + '\n' + 'triangleX ' + str(rtbbYT) + '\n' \
    + 'triangleYB = ' +  str(rtbbYB) +  '\n' + 'triangleArea = ' +  str(rtArea) + '\n')
    
    h.close()
    
#Blue Team data parsing
    
    Blue = data['Blue Team Data']

#Blue Circle
    
    bCircle = Blue['Circle']
    bcCenterPoint = bCircle['Object Center']
    
    #Coordinates
    
    bccpX = bcCenterPoint['X']
    bccpY = bcCenterPoint['Y']
    
    #Bounding Box
    
    bcBoundinBox = bCircle['Bounding Box']
    bcbbXL = bcBoundinBox['X Left']
    bcbbYT = bcBoundinBox['Y Top']
    bcbbXR = bcBoundinBox['X Right']
    bcbbYB = bcBoundinBox['Y Bottom']
    
    #Area
    
    bcArea = bCircle['Area']
    
#Blue Square
    
    bSquare = Blue['Square']
    bsCenterPoint = bSquare['Object Center']
    
    #Coordinates
    
    bscpX = bsCenterPoint['X']
    bscpY = bsCenterPoint['Y']
    
    #Bounding Box
    
    bsBoundinBox = bSquare['Bounding Box']
    bsbbXL = bsBoundinBox['X Left']
    bsbbYT = bsBoundinBox['Y Top']
    bsbbXR = bsBoundinBox['X Right']
    bsbbYB = bsBoundinBox['Y Bottom']
    
    #Area
    
    bsArea = bSquare['Area']
    
#Blue Triangle
    
    bTriangle = Blue['Triangle']
    btCenterPoint = bTriangle['Object Center']
    
    #Coordinates
    
    btcpX = btCenterPoint['X']
    btcpY = btCenterPoint['Y']
    
    #Bounding Box
    
    btBoundinBox = bSquare['Bounding Box']
    btbbXL = btBoundinBox['X Left']
    btbbYT = btBoundinBox['Y Top']
    btbbXR = btBoundinBox['X Right']
    btbbYB = btBoundinBox['Y Bottom']
    
    #Area
    
    btArea = bTriangle['Area']

    #Write Blue team data to ROS message (.msg)

    i = open( 'blue_team_coordinates.msg', 'w' )
    i.write( 'circleX ' + str(bccpX) + '\n' + 'circleY = ' +  str(bccpY) + '\n' + 'circleBBXL ' + str(bcbbXL) + '\n' + 'circleBBXR = ' +  str(bcbbXR) + '\n' + 'circleBBYT ' + str(bcbbYT) + '\n' \
    + 'circleBBYB = ' +  str(bcbbYB) + '\n' + '/n' + 'CircleArea = ' +  str(bcArea) + '\n' +'squareX = ' + str(bscpX) + '\n' + 'squareY = ' +  str(bscpY) + '\n' + 'squareBBXL = ' + str(bsbbXL) + \
    '\n' + 'squareBBXR = ' +  str(bsbbXR) + '\n' + 'squareBBYT ' + str(bsbbYT) + '\n' + 'squareBBYB = ' +  str(bsbbYB) + '\n' +  '/n' + 'squareArea = ' +  str(bsArea) + '/n' \
    'triangleX ' + str(btcpX) + '\n' + 'triangleY = ' +  str(btcpY) + '\n' + 'triangleBBXL ' + str(btbbXL) + '\n' + 'triangleBBXR = ' +  str(btbbXR) + '\n' + 'triangleX ' + str(btbbYT) + '\n' \
    + 'triangleYB = ' +  str(btbbYB) +  '\n' + 'triangleArea = ' +  str(btArea) + '\n')
    i.close()

    print(json.dumps(cornerBRX, indent=2))
    time.sleep(2)

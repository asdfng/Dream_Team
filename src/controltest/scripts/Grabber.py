#Adapted from Noah's json_grabber code
import json, urllib2


response = urllib2.urlopen('http://192.168.137.1:8001/FieldData/GetData')
source = response.read()
data = json.loads(source.decode())
locations = {bSquare: {'X': data['Blue Team Data']['Square']['Object Center']['X'], 'Y': data['Blue Team Data']['Square']['Object Center']['Y']},
            bCircle: {'X': data['Blue Team Data']['Circle']['Object Center']['X'], 'Y': data['Blue Team Data']['Circle']['Object Center']['Y']},
            bTriangle: {'X': data['Blue Team Data']['Triangle']['Object Center']['X'], 'Y': data['Blue Team Data']['Triangle']['Triangle']['Object Center']['Y']},
            rSquare: {'X': data['Red Team Data']['Square']['Object Center']['X'], 'Y': data['Red Team Data']['Square']['Object Center']['Y']},
            rCircle: {'X': data['Red Team Data']['Circle']['Object Center']['X'], 'Y': data['Red Team Data']['Circle']['Object Center']['Y']},
            rTriangle: {'X': data['Red Team Data']['Triangle']['Object Center']['X'], 'Y': data['Red Team Data']['Triangle']['Triangle']['Object Center']['Y']},
            ball: {'X': data['Ball']['Object Center']['X'], 'Y': data['Ball']['Object Center']['Y']}}

print(locations)
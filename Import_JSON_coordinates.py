import json, urllib.request

with urllib.request.urlopen('http://172.16.0.1:8001/FieldData/GetData') as response:
    source = response.read()

data = json.loads(source.decode())

print(json.dumps(data, indent=2))

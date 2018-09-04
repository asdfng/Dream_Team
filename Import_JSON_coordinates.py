import json, urllib.request, time

while 1:

    with urllib.request.urlopen('http://172.16.0.1:8001/FieldData/GetData') as response:
        source = response.read()

    data = json.loads(source.decode())

    print(json.dumps(data, indent=2))
    time.sleep(2)
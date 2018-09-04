i = 0
while i < 180:
    import json, urllib.request, time

    with urllib.request.urlopen('http://172.16.0.1:8001/FieldData/GetData') as response:
        source = response.read()

    data = json.loads(source.decode())

    print(json.dumps(data, indent=2))
    
    i += 1

    time.sleep(1)
<<<<<<< HEAD
i = 0
while i < 180:
    import json, urllib.request, time

    with urllib.request.urlopen('http://172.16.0.1:8001/FieldData/GetData') as response:
        source = response.read()

    data = json.loads(source.decode())

    print(json.dumps(data, indent=2))
    
    i += 1

    time.sleep(1)
=======
import json, urllib.request, time

while 1:

    with urllib.request.urlopen('http://172.16.0.1:8001/FieldData/GetData') as response:
        source = response.read()

    data = json.loads(source.decode())

    print(json.dumps(data, indent=2))
    time.sleep(2)
>>>>>>> 6ba4dd3b558540eef91ea9822e55d4a7e8686898

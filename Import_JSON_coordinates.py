import urllib.request, json 
with urllib.request.urlopen("http://172.16.0.1:8001/FieldData/GetData") as url:
    data = json.loads(url.read().decode())
    print(data)
    

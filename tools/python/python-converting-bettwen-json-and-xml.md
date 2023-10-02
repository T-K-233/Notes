# Python converting bettwen JSON and XML

```bash
pip install cc-xmljson
```



```python
import xml
import json
from cc.xmljson import XMLJSON

def xml2json(in_file):
    xml_data = ""
    with open(in_file, "r") as f:
        xml_data = f.read()

    json_data = XMLJSON.gdata.data(xml.etree.ElementTree.fromstring(xml_data))
    return json_data
    
def json2xml(in_file):
    json_data = json.load(open(in_file, "r"))
    
    xml_data = XMLJSON.gdata.etree(json_data, root=xml.etree.ElementTree.Element("root"))
    xml_data = xml.etree.ElementTree.tostring(xml_data, encoding="utf8")
    return xml_data

print(xml2json("tests/abdera-1.xml"))
print(json2xml("tests/abdera-1.json"))


```

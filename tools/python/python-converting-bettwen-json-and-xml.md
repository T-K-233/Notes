# Python converting bettwen JSON and XML

```python
def xml2json(out_file, in_file):
    xml_data = ""
    with open(in_file, "r") as f:
        xml_data = f.read()

    json_data = xmljson.gdata.data(xml.etree.ElementTree.fromstring(xml_data))
    json.dump(json_data, open(out_file, "w"))
    
def json2xml(out_file, in_file):
    json_data = json.load(open(in_file, "r"))
    
    xml_data = xmljson.gdata.etree(json_data, root=xml.etree.ElementTree.Element("root"))
    xml_data = xml.etree.ElementTree.tostring(xml_data, encoding="utf8")
    with open(out_file, "wb") as f:
        f.write(xml_data)

```

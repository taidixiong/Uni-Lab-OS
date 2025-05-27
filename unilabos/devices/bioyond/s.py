import json
params = json.load(open("/Users/xiongyanfei/Desktop/code/dp/Uni-Lab-OS/unilabos/devices/bioyong/param.txt"))

res = {}
for k, pv in params.items():
    l = []
    for v in pv:
        for item in v["parameterList"]:
            value = None if item["value"] is None else str(item["value"])
            l.append({
                "key": item["key"],
                "value": value,
                "m": item["m"],
                "n": item["n"]
            })
        
    res[k] = l
    
# print(json.dumps(res))
from pymongo import MongoClient
client = MongoClient("mongodb://admin:z4hr4n523@localhost:27017/")
db = client["sortamanggis_db"]
print(list(db["monitoring_sortasi"].find().sort("_id", -1).limit(1)))

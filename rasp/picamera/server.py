from flask import Flask,request, jsonify, send_file
from pymongo import MongoClient
from bson import ObjectId
import subprocess

app = Flask(__name__)
client = MongoClient("mongodb://admin:z4hr4n523@localhost:27017/")
db = client["sortamanggis_db"]
col = db["monitoring_sortasi"]

@app.route("/api/status")
def get_status():
    try:
        # cek apakah proses picamera.py sedang berjalan
        result = subprocess.run(["pgrep", "-f", "ta_picamera_stepper.py"], capture_output=True, text=True)
        if result.stdout.strip():  # ada pid berarti berjalan
            return jsonify({"running": True})
        else:
            return jsonify({"running": False})
    except Exception as e:
        return jsonify({"running": False, "error": str(e)})
    
@app.route("/api/realtime")
def get_latest():
    latest = col.find().sort("_id", -1).limit(1)[0]
    latest["_id"] = str(latest["_id"])
    return jsonify(latest)

@app.route("/api/riwayat")
def get_riwayat():
    data = list(col.find().sort("_id", 1))
    for item in data:
        item["_id"] = str(item["_id"])
    return jsonify(data)

@app.route('/api/delete', methods=['POST'])
def delete_data():
    ids = request.json.get('ids', [])
    object_ids = [ObjectId(id) for id in ids]
    result = col.delete_many({'_id': {'$in': object_ids}})
    return jsonify({'deleted': result.deleted_count})

@app.route("/") 
def serve_app():
    return send_file("app.html")

@app.route("/riwayat")
def serve_riwayat():
    return send_file("riwayat.html")

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)

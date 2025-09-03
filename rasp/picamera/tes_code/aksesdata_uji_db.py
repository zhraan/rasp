from pymongo import MongoClient
import pandas as pd

# Koneksi MongoDB
client = MongoClient("mongodb://admin:z4hr4n523@localhost:27017/")
db = client["sortamanggis_db"]
col = db["ringkasan_uji_kecepatan_db"]

# Ambil semua data
data = list(col.find({}, {"_id": 0}))  # hilangkan _id biar rapi
df = pd.DataFrame(data)

print(df)

# Kalau mau simpan ke CSV
df.to_csv("hasil_ringkasan_uji.csv", index=False)
print("Hasil ringkasan sudah disimpan ke hasil_ringkasan_uji.csv")

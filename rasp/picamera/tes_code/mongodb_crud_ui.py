import tkinter as tk
from tkinter import messagebox, simpledialog
from pymongo import MongoClient
from bson.objectid import ObjectId
from datetime import datetime
import random

# === Koneksi MongoDB ===
client = MongoClient("mongodb://localhost:27017/")
db = client["sortamanggis_db"]
collection = db["monitoring_sortasi"]

# === Fungsi CRUD ===
def create_data():
    now = datetime.now()
    data = {
        "timestamp": now.isoformat(),
        "durasi_sortasi": "00:10:00",
        "kelas_super": random.randint(0, 10),
        "kelas_a": random.randint(0, 10),
        "kelas_b": random.randint(0, 10),
        "rusak": random.randint(0, 5)
    }
    result = collection.insert_one(data)
    messagebox.showinfo("CREATE", f"Data ditambahkan dengan _id:\n{result.inserted_id}")

def read_data():
    docs = list(collection.find().sort("timestamp", -1).limit(10))
    output.delete(1.0, tk.END)
    for doc in docs:
        output.insert(tk.END, f"{doc['_id']} | {doc['timestamp']} | Total: {doc['kelas_super'] + doc['kelas_a'] + doc['kelas_b'] + doc['rusak']}\n")

def update_data():
    _id = simpledialog.askstring("Update", "Masukkan ObjectId dokumen:")
    try:
        new_value = int(simpledialog.askstring("Update", "Jumlah kelas_super baru:"))
        result = collection.update_one(
            {"_id": ObjectId(_id)},
            {"$set": {"kelas_super": new_value}}
        )
        if result.modified_count:
            messagebox.showinfo("UPDATE", "Data berhasil diperbarui.")
        else:
            messagebox.showwarning("UPDATE", "Tidak ada perubahan.")
    except Exception as e:
        messagebox.showerror("UPDATE ERROR", str(e))

def delete_data():
    _id = simpledialog.askstring("Delete", "Masukkan ObjectId dokumen:")
    try:
        result = collection.delete_one({"_id": ObjectId(_id)})
        if result.deleted_count:
            messagebox.showinfo("DELETE", "Data berhasil dihapus.")
        else:
            messagebox.showwarning("DELETE", "Data tidak ditemukan.")
    except Exception as e:
        messagebox.showerror("DELETE ERROR", str(e))

# === GUI Tkinter ===
root = tk.Tk()
root.title("MongoDB CRUD - SortaManggis")
root.geometry("600x400")

frame = tk.Frame(root)
frame.pack(pady=10)

tk.Button(frame, text="CREATE", width=15, command=create_data).grid(row=0, column=0, padx=5)
tk.Button(frame, text="READ", width=15, command=read_data).grid(row=0, column=1, padx=5)
tk.Button(frame, text="UPDATE", width=15, command=update_data).grid(row=0, column=2, padx=5)
tk.Button(frame, text="DELETE", width=15, command=delete_data).grid(row=0, column=3, padx=5)

output = tk.Text(root, height=15, width=70)
output.pack(pady=10)

root.mainloop()

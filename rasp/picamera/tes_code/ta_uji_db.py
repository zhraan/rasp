from pymongo import MongoClient
from pymongo.write_concern import WriteConcern
from datetime import datetime
from time import perf_counter_ns
import statistics

# === Setup MongoDB ===
client = MongoClient("mongodb://admin:z4hr4n523@localhost:27017/")
db = client["sortamanggis_db"]

uji_detail_col = db.get_collection("uji_kecepatan_db").with_options(
    write_concern=WriteConcern(w=1, j=True)
)
uji_ringkas_col = db.get_collection("ringkasan_uji_kecepatan_db").with_options(
    write_concern=WriteConcern(w=1, j=True)
)

# === Fungsi Insert dengan Timing ===
def timed_insert(doc: dict) -> dict:
    t_start_ns = perf_counter_ns()
    t_start = datetime.now()
    res = uji_detail_col.insert_one(doc)
    t_end_ns = perf_counter_ns()
    t_end = datetime.now()
    delay_ms = (t_end_ns - t_start_ns) / 1e6  # konversi ns ? ms

    # Update dokumen dengan metadata waktu
    uji_detail_col.update_one(
        {"_id": res.inserted_id},
        {"$set": {"t_start": t_start, "t_end": t_end, "delay_ms": delay_ms}}
    )

    return {"t_start": t_start, "t_end": t_end, "delay_ms": delay_ms}

# === Fungsi Jalankan Uji Kecepatan ===
def jalankan_uji_kecepatan(batch_sizes=(1, 10, 50, 100, 200)):
    print("\n[UJI-DB] Mulai pengujian kecepatan penyimpanan MongoDB...\n")
    no = 1
    for n in batch_sizes:
        print(f"[UJI-DB] Batch #{no} - mengirim {n} dokumen ...")
        delays = []
        batch_t0 = datetime.now()

        for i in range(n):
            payload = {
                "jenis": "event_deteksi",
                "kelas": "contoh",
                "confidence": 0.99,
                "timestamp": datetime.now()
            }
            met = timed_insert(payload)
            delays.append(met["delay_ms"])

        batch_t1 = datetime.now()

        # Statistik
        avg_ms = statistics.mean(delays)
        p95_ms = statistics.quantiles(delays, n=20)[18] if len(delays) >= 20 else max(delays)
        worst_ms = max(delays)
        best_ms = min(delays)

        # Simpan ringkasan batch
        ringkas = {
            "no": no,
            "jumlah_data_dikirim": n,
            "waktu_kirim": batch_t0,
            "waktu_selesai": batch_t1,
            "delay_rata2_ms": round(avg_ms, 2),
            "delay_p95_ms": round(p95_ms, 2),
            "delay_terbaik_ms": round(best_ms, 2),
            "delay_terburuk_ms": round(worst_ms, 2),
        }
        uji_ringkas_col.insert_one(ringkas)

        # Cetak hasil
        print(
            f"[UJI-DB][{no}] N={n} | Kirim={batch_t0.strftime('%H:%M:%S')} "
            f"| Selesai={batch_t1.strftime('%H:%M:%S')} "
            f"| Avg={avg_ms:.2f} ms | P95={p95_ms:.2f} ms | Min={best_ms:.2f} ms | Max={worst_ms:.2f} ms"
        )
        no += 1

    print("\n[UJI-DB] Pengujian selesai.\n")

# === Jalankan Uji ===
if __name__ == "__main__":
    jalankan_uji_kecepatan(batch_sizes=(1, 10, 50, 100, 200))
import zipfile, pathlib, sys

pptx_path = pathlib.Path(sys.argv[1]).resolve()
out_dir = pptx_path.with_suffix('').parent / (pptx_path.stem + "_media")
out_dir.mkdir(exist_ok=True)

with zipfile.ZipFile(pptx_path) as z:
    for member in z.namelist():
        if member.startswith("ppt/media/"):
            data = z.read(member)
            fname = pathlib.Path(member).name  # e.g., video1.mp4
            (out_dir / fname).write_bytes(data)
            print("Extracted:", out_dir / fname)

print("Done. Media saved to:", out_dir)

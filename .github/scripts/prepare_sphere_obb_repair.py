from pathlib import Path

path = Path(".github/scripts/repair_sphere_obb_rational.py")
text = path.read_text()
replacements = {
    '"fn position_delta(\\n"': '"fn position_delta(center: Position, point: Position)"',
    '"fn cross_i64_i128(\\n"': '"fn cross_i64_i128(left: [i64; 3], right: [i128; 3]) -> Result<[i128; 3], SphereObbResponseError3d> {"',
}
for old, new in replacements.items():
    if old not in text:
        raise SystemExit(f"repair driver marker missing: {old}")
    text = text.replace(old, new, 1)
path.write_text(text)

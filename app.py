"""
PyCAD Studio - Parametric 3D CAD & Slicer Prep Backend
Powered by Flask, CadQuery (OpenCASCADE), and Python
"""

import os
import tempfile
import traceback
from flask import Flask, request, jsonify, send_file
from flask_cors import CORS
import cadquery as cq

app = Flask(__name__, static_folder=".", static_url_path="")
CORS(app)


def generate_enclosure(length: float, width: float, height: float,
                       wall: float, fillet: float, hole: float, flange: bool):
    """
    Parametric Electronic Enclosure / Project Box.
    Hollow internal cavity with reinforced floor, vertical corner fillets,
    and optional outer mounting flange tabs with screw holes.
    """
    # Safe dimensions calculation
    wall = max(1.0, min(wall, min(length, width) / 4))
    safe_fillet = min(fillet, (min(length, width) / 2) - wall - 0.5, wall * 2.5)
    safe_fillet = max(0.5, safe_fillet)

    # Outer solid box
    box = cq.Workplane("XY").box(length, width, height)
    if safe_fillet > 0.5:
        box = box.edges("|Z").fillet(safe_fillet)

    # Inner hollow cavity (open at top Z)
    cavity_len = max(2.0, length - 2 * wall)
    cavity_wid = max(2.0, width - 2 * wall)
    cavity_h = height - wall

    cavity = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, wall / 2))
        .box(cavity_len, cavity_wid, cavity_h)
    )
    if safe_fillet > wall and (safe_fillet - wall) > 0.5:
        cavity = cavity.edges("|Z").fillet(safe_fillet - wall)

    body = box.cut(cavity)

    # Optional Corner Screw Posts inside the cavity
    if hole > 0 and cavity_len > hole * 4 and cavity_wid > hole * 4:
        boss_od = max(hole + 2.4, hole * 1.8)
        boss_r = boss_od / 2
        post_h = cavity_h - 1.0  # slight clearance below rim
        px = cavity_len / 2 - boss_r
        py = cavity_wid / 2 - boss_r
        pz = -height / 2 + wall + post_h / 2

        posts_plane = cq.Workplane("XY").transformed(offset=(0, 0, pz))
        posts = (
            posts_plane.pushPoints([(px, py), (-px, py), (px, -py), (-px, -py)])
            .circle(boss_r)
            .extrude(post_h)
        )
        body = body.union(posts)

        # Pilot screw holes in bosses
        holes = (
            posts_plane.pushPoints([(px, py), (-px, py), (px, -py), (-px, -py)])
            .circle(hole / 2)
            .extrude(post_h * 1.5)
        )
        body = body.cut(holes)

    # Optional External Mounting Flange Ears
    if flange:
        flange_w = width * 0.7
        flange_t = wall
        flange_ext = 14.0
        ear_z = -height / 2 + flange_t / 2

        ear1 = (
            cq.Workplane("XY")
            .transformed(offset=(-length / 2 - flange_ext / 2, 0, ear_z))
            .box(flange_ext, flange_w, flange_t)
        )
        ear2 = (
            cq.Workplane("XY")
            .transformed(offset=(length / 2 + flange_ext / 2, 0, ear_z))
            .box(flange_ext, flange_w, flange_t)
        )
        body = body.union(ear1).union(ear2)

        # Screw mounting holes in ears
        if hole > 0:
            h1 = (
                cq.Workplane("XY")
                .transformed(offset=(-length / 2 - flange_ext / 2, 0, ear_z))
                .circle(hole / 2)
                .extrude(flange_t * 2)
                .translate((0, 0, -flange_t / 2))
            )
            h2 = (
                cq.Workplane("XY")
                .transformed(offset=(length / 2 + flange_ext / 2, 0, ear_z))
                .circle(hole / 2)
                .extrude(flange_t * 2)
                .translate((0, 0, -flange_t / 2))
            )
            body = body.cut(h1).cut(h2)

    return body


def generate_bracket(length: float, width: float, height: float,
                     wall: float, fillet: float, hole: float, flange: bool):
    """
    Parametric 90-degree Structural L-Bracket.
    With center stiffening triangular rib/gusset and countersunk/through screw holes.
    """
    wall = max(1.5, min(wall, min(length, height) / 3))

    # Base horizontal leg along +X
    base = cq.Workplane("XY").box(length, width, wall).translate((length / 2, 0, wall / 2))
    # Vertical leg along +Z
    upright = cq.Workplane("XY").box(wall, width, height).translate((wall / 2, 0, height / 2))
    bracket = base.union(upright)

    # Optional Stiffener / Gusset web
    if flange and min(length, height) > wall * 2:
        gusset_len = min(length * 0.5, height * 0.5, 30.0)
        gusset_t = min(wall, 4.0)
        pts = [(wall, wall), (wall + gusset_len, wall), (wall, wall + gusset_len)]
        gusset = (
            cq.Workplane("XZ")
            .transformed(offset=(0, 0, -gusset_t / 2))
            .polyline(pts)
            .close()
            .extrude(gusset_t)
        )
        bracket = bracket.union(gusset)

    # Mounting holes on both legs
    if hole > 0:
        safe_hole = min(hole, width * 0.6)
        # Base hole
        h1 = (
            cq.Workplane("XY")
            .transformed(offset=(length * 0.65, 0, 0))
            .circle(safe_hole / 2)
            .extrude(wall * 3)
            .translate((0, 0, -wall))
        )
        # Upright hole
        h2 = (
            cq.Workplane("YZ")
            .transformed(offset=(0, height * 0.65, 0))
            .circle(safe_hole / 2)
            .extrude(wall * 3)
            .translate((-wall, 0, 0))
        )
        bracket = bracket.cut(h1).cut(h2)

    return bracket


def generate_standoff(length: float, width: float, height: float,
                      wall: float, fillet: float, hole: float, flange: bool):
    """
    Parametric Standoff, Spacer, Bushing or Flanged Pin.
    Cylindrical body with inner through-bore and optional hexagonal mounting flange base.
    """
    outer_r = max(3.0, width / 2)
    body = cq.Workplane("XY").circle(outer_r).extrude(height)

    # Hex base flange
    if flange:
        flange_r = outer_r * 1.35
        flange_h = min(height * 0.3, 5.0)
        flange_base = cq.Workplane("XY").polygon(6, flange_r * 2).extrude(flange_h)
        body = body.union(flange_base)

    # Inner bore hole
    if hole > 0:
        safe_hole = min(hole, (outer_r * 2) - (wall * 1.2))
        if safe_hole > 0.5:
            bore = (
                cq.Workplane("XY")
                .circle(safe_hole / 2)
                .extrude(height * 2)
                .translate((0, 0, -height * 0.5))
            )
            body = body.cut(bore)

    return body


@app.route("/")
def serve_index():
    """Serve the React PyCAD Studio frontend."""
    return send_file("index.html")


@app.route("/api/health", methods=["GET"])
def health_check():
    """Health check endpoint to verify backend connectivity."""
    return jsonify({
        "status": "healthy",
        "engine": "CadQuery (OpenCASCADE)",
        "version": getattr(cq, "__version__", "2.x"),
        "supportedFormats": ["STL", "STEP"],
        "supportedTemplates": ["enclosure", "bracket", "standoff"]
    })


@app.route("/api/generate", methods=["POST"])
def generate_cad():
    """
    Generate CAD geometry based on parameters and export as STL or STEP.
    """
    try:
        data = request.get_json(force=True) or {}

        model_type = data.get("modelType", "enclosure")
        length = float(data.get("length", 60.0))
        width = float(data.get("width", 40.0))
        height = float(data.get("height", 25.0))
        wall = float(data.get("wallThickness", 2.4))
        fillet = float(data.get("filletRadius", 3.0))
        hole = float(data.get("holeDiameter", 3.2))
        flange = bool(data.get("addFlange", True))
        export_format = str(data.get("format", "STL")).upper()

        if model_type == "enclosure":
            model = generate_enclosure(length, width, height, wall, fillet, hole, flange)
        elif model_type == "bracket":
            model = generate_bracket(length, width, height, wall, fillet, hole, flange)
        elif model_type == "standoff":
            model = generate_standoff(length, width, height, wall, fillet, hole, flange)
        else:
            return jsonify({"error": f"Unknown model type '{model_type}'"}), 400

        # Create temporary file for exporter output
        ext = ".stl" if export_format == "STL" else ".step"
        temp_dir = tempfile.gettempdir()
        temp_path = os.path.join(temp_dir, f"pycad_{os.getpid()}_{model_type}{ext}")

        if export_format == "STL":
            cq.exporters.export(model, temp_path, "STL", tolerance=0.01, angularTolerance=0.1)
            mimetype = "application/sla"
        elif export_format == "STEP":
            cq.exporters.export(model, temp_path, "STEP")
            mimetype = "application/step"
        else:
            return jsonify({"error": f"Unsupported format '{export_format}'"}), 400

        filename = f"{model_type}_{int(length)}x{int(width)}x{int(height)}{ext}"

        response = send_file(
            temp_path,
            mimetype=mimetype,
            as_attachment=True,
            download_name=filename
        )
        response.headers["Access-Control-Expose-Headers"] = "Content-Disposition, X-Model-Type"
        response.headers["X-Model-Type"] = model_type
        return response

    except Exception as exc:
        traceback.print_exc()
        return jsonify({
            "error": "Failed to generate CAD model",
            "details": str(exc)
        }), 500


if __name__ == "__main__":
    port = int(os.environ.get("PORT", 5000))
    print(f"Starting PyCAD Studio backend at http://127.0.0.1:{port}")
    app.run(host="127.0.0.1", port=port, debug=True)
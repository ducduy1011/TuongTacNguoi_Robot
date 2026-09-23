"""Generate a standalone LaTeX report with verbatim local source excerpts."""
from pathlib import Path
import hashlib
import json
import re

HERE = Path(__file__).resolve().parent
CIRCLE = HERE.parent
LETTER = CIRCLE.parent / "ur3_letter_writer"
letter = (LETTER / "src/draw_letter_d.cpp").read_text()
circle = (CIRCLE / "src/draw_circle.cpp").read_text()


def between(source, start, end):
    begin = source.index(start)
    finish = source.index(end, begin)
    return source[begin:finish].rstrip()


def function(source, name):
    start = source.index("std::vector<geometry_msgs::msg::Pose> " + name)
    end = source.index("\n}", start) + 2
    return source[start:end]


blocks = {}


def add(key, body, caption, path, language="C++"):
    source = path.read_text()
    assert body in source, key
    # Preserve source indentation, comments and whitespace in all excerpts.
    blocks[key] = (body, caption, path, language)


lp = LETTER / "src/draw_letter_d.cpp"
cp = CIRCLE / "src/draw_circle.cpp"
add("LETTER_FUNCTION", function(letter, "createLetterD"), "Hàm sinh waypoint chữ D", lp)
add("CIRCLE_FUNCTION", function(circle, "createCircle"), "Hàm sinh waypoint hình tròn", cp)
add("MOVEIT", between(circle, "    moveit::planning_interface::MoveGroupInterface move_group(", "\n\n    // --------------------------------------------------------\n    // PRE-DRAW"), "Khởi tạo MoveIt và kiểm tra trạng thái robot", cp)
add("READY", between(circle, "    move_group.setStartStateToCurrentState();", "\n\n    // --------------------------------------------------------\n    // TAO HINH TRON"), "Di chuyển tới PRE-DRAW và lấy pose gốc", cp)
add("CIRCLE_PATH", between(circle, "    moveit_msgs::msg::RobotTrajectory trajectory;", "\n\n    // --------------------------------------------------------\n    // RVIZ GUIDE"), "Tính và kiểm tra Cartesian path hình tròn", cp)
add("LETTER_PATH", between(letter, "    const auto positive =", "\n\n    // --------------------------------------------------------\n    // CHON HUONG TOT HON"), "Tạo và tính Cartesian path cho hai hướng chữ D", lp)
add("LETTER_SELECT", between(letter, "    const bool use_positive =", "\n\n    // --------------------------------------------------------\n    // RVIZ GUIDE"), "Chọn trajectory, waypoint và kiểm tra fraction chữ D", lp)
for key, source, path, caption in [
    ("LETTER_LOOP", letter, lp, "Vòng lặp thực thi trajectory chữ D"),
    ("CIRCLE_LOOP", circle, cp, "Vòng lặp thực thi trajectory hình tròn"),
]:
    add(key, between(source, "    int count = 1;", "\n\n    shutdown();\n    return 0;"), caption, path)
for key, path, caption in [
    ("LETTER_YAML", LETTER / "config/letter_d.yaml", r"Cấu hình letter\_d.yaml"),
    ("CIRCLE_YAML", CIRCLE / "config/circle.yaml", "Cấu hình circle.yaml"),
]:
    add(key, path.read_text().rstrip(), caption, path, "")

template = (HERE / "BaoCaoT1_template.tex").read_text()
manifest = []
for key, (body, caption, path, language) in blocks.items():
    token = "@@" + key + "@@"
    assert template.count(token) == 1, token
    listing = "\\begin{lstlisting}[language={" + language + "},caption={" + caption + "}]\n" + body + "\n\\end{lstlisting}"
    template = template.replace(token, listing)
    source = path.read_text()
    manifest.append({
        "block": key,
        "source": str(path),
        "first_line": source[:source.index(body)].count("\n") + 1,
        "source_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "excerpt_sha256": hashlib.sha256(body.encode()).hexdigest(),
    })
assert not re.search(r"@@[A-Z_]+@@", template)
assert template.count("\\begin{lstlisting}") == template.count("\\end{lstlisting}")
(HERE / "BaoCaoT1_da_doi_chieu.tex").write_text(template)
(HERE / "source_manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n")
print(f"Generated standalone report with {len(blocks)} exact source excerpts.")

import streamlit as st
from reportlab.pdfgen import canvas
from reportlab.lib.units import inch
import cairosvg
import tempfile
import os
import zipfile
from io import BytesIO

def svg_to_pdf(svg_bytes, output_pdf_path):
    # Convert SVG to PNG with white background
    with tempfile.NamedTemporaryFile(delete=False, suffix=".png") as temp_png:
        cairosvg.svg2png(
            bytestring=svg_bytes,
            write_to=temp_png.name,
            output_width=int(150 / 25.4 * 96),
            output_height=int(150 / 25.4 * 96),
            background_color='white'
        )

        # Create a PDF with the image centered
        c = canvas.Canvas(output_pdf_path, pagesize=(8.5 * inch, 11 * inch))
        x_offset = (8.5 - 5.91) / 2 * inch
        y_offset = (11 - 5.91) / 2 * inch
        c.drawImage(temp_png.name, x_offset, y_offset, width=5.91 * inch, height=5.91 * inch)
        c.showPage()
        c.save()

# --- Streamlit UI ---
st.title("🧩 SVG to Printable PDF (15cm × 15cm)")

uploaded_files = st.file_uploader("Upload one or more SVG files", type="svg", accept_multiple_files=True)

if uploaded_files:
    if len(uploaded_files) == 1:
        st.subheader("📄 Single File Detected")
        file = uploaded_files[0]
        default_name = os.path.splitext(file.name)[0]
        custom_name = st.text_input("Name your board", value=default_name).strip().replace(" ", "_")
        if st.button("Convert and Download PDF"):
            with tempfile.NamedTemporaryFile(delete=False, suffix=".pdf") as pdf_temp:
                svg_to_pdf(file.read(), pdf_temp.name)
                with open(pdf_temp.name, "rb") as f:
                    st.download_button(
                        label="📥 Download PDF",
                        data=f,
                        file_name=f"{custom_name}_board.pdf",
                        mime="application/pdf"
                    )
                os.remove(pdf_temp.name)
    else:
        st.subheader("📦 Multiple Files Detected — Customize and Download ZIP")

        custom_names = {}
        aliased_files = []
        for idx, file in enumerate(uploaded_files, start=1):
            alias = f"svg{idx}.svg"
            aliased_files.append((alias, file))

        for alias, file in aliased_files:
            default_name = os.path.splitext(alias)[0]
            name = st.text_input(f"Name for '{alias}'", value=default_name)
            custom_names[alias] = (name.strip().replace(" ", "_"), file)

        if st.button("Convert and Download All as ZIP"):
            zip_buffer = BytesIO()
            with zipfile.ZipFile(zip_buffer, "w") as zip_file:
                for alias, (custom_name, file) in custom_names.items():
                    pdf_name = f"{custom_name}_board.pdf"
                    with tempfile.NamedTemporaryFile(delete=False, suffix=".pdf") as pdf_temp:
                        svg_to_pdf(file.read(), pdf_temp.name)
                        zip_file.write(pdf_temp.name, arcname=pdf_name)
                        os.remove(pdf_temp.name)
            zip_buffer.seek(0)
            st.download_button("📥 Download ZIP of All Boards", zip_buffer, file_name="boards.zip", mime="application/zip")
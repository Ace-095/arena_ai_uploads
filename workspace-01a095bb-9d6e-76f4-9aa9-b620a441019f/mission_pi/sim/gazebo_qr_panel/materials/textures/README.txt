Put qr.png here (the panel texture Gazebo renders):

  cd mission_pi
  python tools/make_qr_panel.py --text MISSION-QR-001 \
      --out sim/gazebo_qr_panel/materials/textures/qr.png

PNG is git-ignored by design (regenerate any time); any A3-ratio QR
image with this filename works.

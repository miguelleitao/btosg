
center.png:
	convert -size 256x256 -define gradient:center=100,80 -define gradient:radii=160,160 radial-gradient:white-black -colorspace RGB -colorspace gray -depth 8 $@

spiral_c.png: center.png spiral_i.png
	convert $< $^ -average -blur 8x8 $@ 

spiral_i.png: spiral.png
	convert $< -blur 8x8 -negate $@


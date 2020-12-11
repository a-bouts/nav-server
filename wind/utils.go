package wind

func Twa(heading, wind float64) float64 {
	twa := wind - heading
	if twa <= -180 {
		twa += 360
	}
	if twa > 180 {
		twa -= 360
	}

	return twa
}

func Heading(twa, wind float64) float64 {
	heading := wind - twa
	if heading < 0 {
		heading += 360
	}
	if heading >= 360 {
		heading -= 360
	}

	return heading
}

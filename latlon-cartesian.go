package main

import "math"

type LatLonCartesian struct{}

func (LatLonCartesian) initialBearingTo(from, to LatLon) float64 {
	x := to.Lon - from.Lon
	y := to.Lat - from.Lat

	if x > 180 {
		x -= 360
	} else if x < -180 {
		x += 360
	}

	d := math.Sqrt(x*x + y*y)

	α := math.Acos(y / d)
	if x < 0 {
		α *= -1
	}

	b := toDegrees(α)

	return wrap360(b)
}

func (LatLonCartesian) DistanceTo(from, to LatLon) float64 {
	x := to.Lon - from.Lon
	y := to.Lat - from.Lat

	if x > 180 {
		x -= 360
	} else if x < -180 {
		x += 360
	}

	d := math.Sqrt(x*x + y*y)

	return d
}

func (LatLonCartesian) BearingTo(from, to LatLon) float64 {
	x := to.Lon - from.Lon
	y := to.Lat - from.Lat

	if x > 180 {
		x -= 360
	} else if x < -180 {
		x += 360
	}

	d := math.Sqrt(x*x + y*y)

	α := math.Acos(y / d)
	if x < 0 {
		α *= -1
	}

	b := toDegrees(α)

	return wrap360(b)
}

func (LatLonCartesian) DistanceAndBearingTo(from, to LatLon) (float64, float64) {
	x := to.Lon - from.Lon
	y := to.Lat - from.Lat

	if x > 180 {
		x -= 360
	} else if x < -180 {
		x += 360
	}

	d := math.Sqrt(x*x + y*y)

	α := math.Acos(y / d)
	if x < 0 {
		α *= -1
	}

	b := toDegrees(α)

	return d, wrap360(b)
}

func (LatLonCartesian) Destination(from LatLon, bearing float64, distance float64) LatLon {

	// TODO : a faire mais sert à rien

	return LatLon{Lat: 0.0, Lon: 0.0}
}

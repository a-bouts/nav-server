package main

import "math"

const π = math.Pi
const R = 6371e3

type LatLonInterface interface {
	DistanceTo(from, to LatLon) float64
	BearingTo(from, to LatLon) float64
	DistanceAndBearingTo(from, to LatLon) (float64, float64)
	Destination(from LatLon, bearing float64, distance float64) LatLon
}

type LatLon struct {
	Lat float64 `json:"lat"`
	Lon float64 `json:"lon"`
	//LatLonSpherical
}

func toRadians(a float64) float64 {
	return a * π / 180.0
}

func toDegrees(a float64) float64 {
	return a * 180.0 / π
}

func wrap360(d float64) float64 {
	if 0.0 <= d && d < 360.0 {
		return d
	}
	//d1 := d - float64(int(d / 360.0) * 360) + 360.0
	d1 := d + 360.0
	d2 := d1 - float64(int(d1/360.0)*360)
	return d2
}

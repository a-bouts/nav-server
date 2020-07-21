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
	LatLonSpherical
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

type LatLonSpherical struct{}

func (LatLonSpherical) initialBearingTo(from, to LatLon) float64 {
	φ1 := toRadians(from.Lat)
	φ2 := toRadians(to.Lat)
	Δλ := toRadians(to.Lon - from.Lon)

	x := math.Cos(φ1)*math.Sin(φ2) - math.Sin(φ1)*math.Cos(φ2)*math.Cos(Δλ)
	y := math.Sin(Δλ) * math.Cos(φ2)
	θ := math.Atan2(y, x)

	b := toDegrees(θ)

	return wrap360(b)
}

func (LatLonSpherical) DistanceTo(from, to LatLon) float64 {
	φ1 := toRadians(from.Lat)
	φ2 := toRadians(to.Lat)
	Δφ := φ2 - φ1

	Δλ := toRadians(math.Abs(to.Lon - from.Lon))
	if math.Abs(Δλ) > π {
		if Δλ > 0 {
			Δλ = -(2*π - Δλ)
		} else {
			Δλ = (2*π + Δλ)
		}
	}

	Δψ := math.Log(math.Tan(φ2/2+π/4) / math.Tan(φ1/2+π/4))

	q := Δφ / Δψ
	if math.Abs(Δψ) <= 10e-12 {
		q = math.Cos(φ1)
	}

	δ := math.Sqrt(Δφ*Δφ + q*q*Δλ*Δλ)
	d := δ * R

	return d
}

func (LatLonSpherical) BearingTo(from, to LatLon) float64 {
	φ1 := toRadians(from.Lat)
	φ2 := toRadians(to.Lat)

	Δλ := toRadians(to.Lon - from.Lon)
	if math.Abs(Δλ) > π {
		if Δλ > 0 {
			Δλ = -(2*π - Δλ)
		} else {
			Δλ = (2*π + Δλ)
		}
	}

	Δψ := math.Log(math.Tan(φ2/2+π/4) / math.Tan(φ1/2+π/4))

	θ := math.Atan2(Δλ, Δψ)

	b := toDegrees(θ)

	return wrap360(b)
}

func (LatLonSpherical) DistanceAndBearingTo(from, to LatLon) (float64, float64) {
	φ1 := toRadians(from.Lat)
	φ2 := toRadians(to.Lat)
	Δφ := φ2 - φ1

	Δλ := toRadians(to.Lon - from.Lon)
	if math.Abs(Δλ) > π {
		if Δλ > 0 {
			Δλ = -(2*π - Δλ)
		} else {
			Δλ = (2*π + Δλ)
		}
	}

	Δψ := math.Log(math.Tan(φ2/2+π/4) / math.Tan(φ1/2+π/4))

	//distance
	q := Δφ / Δψ
	if math.Abs(Δψ) <= 10e-12 {
		q = math.Cos(φ1)
	}

	δ := math.Sqrt(Δφ*Δφ + q*q*Δλ*Δλ)
	d := δ * R

	//bearing
	θ := math.Atan2(Δλ, Δψ)

	b := toDegrees(θ)

	return d, wrap360(b)
}

func (LatLonSpherical) Destination(from LatLon, bearing float64, distance float64) LatLon {
	φ1 := toRadians(from.Lat)
	λ1 := toRadians(from.Lon)
	θ := toRadians(bearing)

	δ := distance / R

	Δφ := δ * math.Cos(θ)
	φ2 := φ1 + Δφ

	if math.Abs(φ2) > π/2 {
		if φ2 > 0 {
			φ2 = π - φ2
		} else {
			φ2 = -π - φ2
		}
	}

	Δψ := math.Log(math.Tan(φ2/2+π/4) / math.Tan(φ1/2+π/4))
	q := Δφ / Δψ
	if math.Abs(Δψ) <= 10e-12 {
		q = math.Cos(φ1)
	}

	Δλ := δ * math.Sin(θ) / q
	λ2 := λ1 + Δλ

	lat := toDegrees(φ2)
	lon := toDegrees(λ2)

	return LatLon{Lat: lat, Lon: lon}
}

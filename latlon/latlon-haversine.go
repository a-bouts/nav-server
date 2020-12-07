package latlon

import "math"

type LatLonHaversine struct{}

func (LatLonHaversine) initialBearingTo(from, to LatLon) float64 {
	φ1 := toRadians(from.Lat)
	φ2 := toRadians(to.Lat)

	Δλ := toRadians(to.Lon - from.Lon)
	x := math.Cos(φ1)*math.Sin(φ2) - math.Sin(φ1)*math.Cos(φ2)*math.Cos(Δλ)
	y := math.Sin(Δλ) * math.Cos(φ2)
	θ := math.Atan2(y, x)

	b := toDegrees(θ)

	return wrap360(b)
}

func (LatLonHaversine) DistanceTo(from, to LatLon) float64 {
	φ1 := toRadians(from.Lat)
	φ2 := toRadians(to.Lat)
	Δφ := φ2 - φ1

	Δλ := toRadians(to.Lon - from.Lon)

	a := math.Sin(Δφ/2)*math.Sin(Δφ/2) + math.Cos(φ1)*math.Cos(φ2)*math.Sin(Δλ/2)*math.Sin(Δλ/2)
	δ := 2 * math.Atan2(math.Sqrt(a), math.Sqrt(1-a))
	d := R * δ

	return d
}

func (hav LatLonHaversine) BearingTo(from, to LatLon) float64 {
	return hav.initialBearingTo(from, to)
}

func (LatLonHaversine) DistanceAndBearingTo(from, to LatLon) (float64, float64) {
	φ1 := toRadians(from.Lat)
	φ2 := toRadians(to.Lat)
	Δφ := φ2 - φ1

	Δλ := toRadians(to.Lon - from.Lon)

	a := math.Sin(Δφ/2)*math.Sin(Δφ/2) + math.Cos(φ1)*math.Cos(φ2)*math.Sin(Δλ/2)*math.Sin(Δλ/2)
	δ := 2 * math.Atan2(math.Sqrt(a), math.Sqrt(1-a))
	d := R * δ

	x := math.Cos(φ1)*math.Sin(φ2) - math.Sin(φ1)*math.Cos(φ2)*math.Cos(Δλ)
	y := math.Sin(Δλ) * math.Cos(φ2)
	θ := math.Atan2(y, x)

	b := toDegrees(θ)

	return d, wrap360(b)
}

func (LatLonHaversine) Destination(from LatLon, bearing float64, distance float64) LatLon {

	// TODO : a faire mais sert à rien

	return LatLon{Lat: 0.0, Lon: 0.0}
}

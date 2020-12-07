package latlon

import "math"

type LatLonZezo struct{}

func (LatLonZezo) DistanceTo(from, to LatLon) float64 {
	φ1 := toRadians(from.Lat)
	φ2 := toRadians(to.Lat)
	Δλ := toRadians(math.Abs(to.Lon - from.Lon))

	δ := math.Acos(math.Sin(φ1)*math.Sin(φ2) + math.Cos(φ1)*math.Cos(φ2)*math.Cos(Δλ))

	d := δ * R

	return d
}

func (LatLonZezo) BearingTo(from, to LatLon) float64 {
	φ1 := toRadians(from.Lat)
	φ2 := toRadians(to.Lat)
	Δλ := toRadians(math.Abs(to.Lon - from.Lon))

	δ := math.Acos(math.Sin(φ1)*math.Sin(φ2) + math.Cos(φ1)*math.Cos(φ2)*math.Cos(Δλ))

	θ := math.Acos((math.Sin(φ2) - math.Sin(φ1)*math.Cos(δ)) / (math.Sin(δ) * math.Cos(φ1)))
	if math.Sin(Δλ) <= 0 {
		θ = 2*math.Pi - θ
	}

	b := toDegrees(θ)

	return wrap360(b)
}

func (LatLonZezo) DistanceAndBearingTo(from, to LatLon) (float64, float64) {
	φ1 := toRadians(from.Lat)
	φ2 := toRadians(to.Lat)
	Δλ := toRadians(math.Abs(to.Lon - from.Lon))

	δ := math.Acos(math.Sin(φ1)*math.Sin(φ2) + math.Cos(φ1)*math.Cos(φ2)*math.Cos(Δλ))

	d := δ * R

	θ := math.Acos((math.Sin(φ2) - math.Sin(φ1)*math.Cos(δ)) / (math.Sin(δ) * math.Cos(φ1)))
	if math.Sin(Δλ) <= 0 {
		θ = 2*math.Pi - θ
	}

	b := toDegrees(θ)

	return d, wrap360(b)
}

func (LatLonZezo) Destination(from LatLon, bearing float64, distance float64) LatLon {
	φ1 := toRadians(from.Lat)
	λ1 := toRadians(from.Lon)
	θ := toRadians(bearing)

	δ := distance / R

	φ2 := math.Asin(math.Sin(φ1)*math.Cos(δ) + math.Cos(φ1)*math.Sin(δ)*math.Cos(θ))
	λ2 := λ1 + math.Atan2(math.Sin(θ)*math.Sin(δ)*math.Cos(φ1), math.Cos(δ)-math.Sin(φ1)*math.Sin(φ2))
	λ2 = (λ2 + math.Pi) - math.Round((λ2+math.Pi)/(2*math.Pi)) - math.Pi

	lat := toDegrees(φ2)
	lon := toDegrees(λ2)

	return LatLon{Lat: lat, Lon: lon}
}

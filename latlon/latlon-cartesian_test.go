package latlon

import (
	"math"
	"testing"
)

func TestBearingTo(t *testing.T) {
	p1 := LatLon{Lat: -5, Lon: -5}
	p2 := LatLon{Lat: 5, Lon: 5}
	d := LatLonCartesian{}.BearingTo(p1, p2)
	if math.Round(d) != 45.0 {
		t.Errorf("{%f,%f}.bearingTo({%f,%f}) = %f; want 45", p1.Lat, p1.Lon, p2.Lat, p2.Lon, d)
	}

	p1 = LatLon{Lat: 5, Lon: 5}
	p2 = LatLon{Lat: 10, Lon: 10}
	d = LatLonCartesian{}.BearingTo(p1, p2)
	if math.Round(d) != 45.0 {
		t.Errorf("{%f,%f}.bearingTo({%f,%f}) = %f; want 45", p1.Lat, p1.Lon, p2.Lat, p2.Lon, d)
	}

	p1 = LatLon{Lat: -10, Lon: -10}
	p2 = LatLon{Lat: -5, Lon: -5}
	d = LatLonCartesian{}.BearingTo(p1, p2)
	if math.Round(d) != 45.0 {
		t.Errorf("{%f,%f}.bearingTo({%f,%f}) = %f; want 45", p1.Lat, p1.Lon, p2.Lat, p2.Lon, d)
	}

	p1 = LatLon{Lat: -5, Lon: 5}
	p2 = LatLon{Lat: 5, Lon: -5}
	d = LatLonCartesian{}.BearingTo(p1, p2)
	if math.Round(d) != 315.0 {
		t.Errorf("{%f,%f}.bearingTo({%f,%f}) = %f; want 315", p1.Lat, p1.Lon, p2.Lat, p2.Lon, d)
	}

	p1 = LatLon{Lat: -5, Lon: 175}
	p2 = LatLon{Lat: 5, Lon: -175}
	d = LatLonCartesian{}.BearingTo(p1, p2)
	if math.Round(d) != 45.0 {
		t.Errorf("{%f,%f}.bearingTo({%f,%f}) = %f; want 45", p1.Lat, p1.Lon, p2.Lat, p2.Lon, d)
	}

	p1 = LatLon{Lat: -5, Lon: -175}
	p2 = LatLon{Lat: 5, Lon: 175}
	d = LatLonCartesian{}.BearingTo(p1, p2)
	if math.Round(d) != 315.0 {
		t.Errorf("{%f,%f}.bearingTo({%f,%f}) = %f; want 315", p1.Lat, p1.Lon, p2.Lat, p2.Lon, d)
	}

}

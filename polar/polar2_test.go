package polar

import (
	"math"
	"testing"
)

func TestInterpolationIndex(t *testing.T) {

	array := []float64{0, 4, 8}

	i0, i1, d := interpolationIndex(array, 0)
	if i0 != 0 || i1 != 0 || d != 0.0 {
		t.Errorf("interpolationIndex(0) = (%d, %d, %f); want (0, 0, 0.0)", i0, i1, d)
	}

	i0, i1, d = interpolationIndex(array, 1)
	if i0 != 0 || i1 != 1 || d != 0.75 {
		t.Errorf("interpolationIndex(1) = (%d, %d, %f); want (0, 1, 0.75)", i0, i1, d)
	}

	i0, i1, d = interpolationIndex(array, 2)
	if i0 != 0 || i1 != 1 || d != 0.5 {
		t.Errorf("interpolationIndex(2) = (%d, %d, %f); want (0, 1, 0.5)", i0, i1, d)
	}

	i0, i1, d = interpolationIndex(array, 3)
	if i0 != 0 || i1 != 1 || d != 0.25 {
		t.Errorf("interpolationIndex(3) = (%d, %d, %f); want (0, 1, 0.25)", i0, i1, d)
	}

	i0, i1, d = interpolationIndex(array, 4)
	if i0 != 0 || i1 != 1 || d != 0.0 {
		t.Errorf("interpolationIndex(4) = (%d, %d, %f); want (0, 1, 0.0)", i0, i1, d)
	}

	i0, i1, d = interpolationIndex(array, 5)
	if i0 != 1 || i1 != 2 || d != 0.75 {
		t.Errorf("interpolationIndex(5) = (%d, %d, %f); want (1, 2, 0.75)", i0, i1, d)
	}

	i0, i1, d = interpolationIndex(array, 8)
	if i0 != 1 || i1 != 2 || d != 0.0 {
		t.Errorf("interpolationIndex(8) = (%d, %d, %f); want (1, 2, 0.0)", i0, i1, d)
	}

	i0, i1, d = interpolationIndex(array, 9)
	if i0 != 2 || i1 != 0 || d != 1.0 {
		t.Errorf("interpolationIndex(9) = (%d, %d, %f); want (2, 0, 1.0)", i0, i1, d)
	}
}

func TestGetBoatSpeed(t *testing.T) {
	z := Load(Options{Race: "arctic", Sail: 0})

	ws, s, _ := z.GetBoatSpeed(0, 0, Boat{})
	if ws != 0 || s != 0 {
		t.Errorf("GetBoatSpeed(0, 0) = (%f, %d); want (0, 0)", ws, s)
	}

	ws, s, _ = z.GetBoatSpeed(60, 39.1, Boat{})
	if math.Round(ws*1000000) != 14600400 || s != 2 {
		t.Errorf("GetBoatSpeed(60, 39.1) = (%f, %d); want (14.600400, 2)", ws, s)
	}

	ws, s, _ = z.GetBoatSpeed(60, 39.1, Boat{Hull: true})
	if math.Round(ws*1000000) != 14644201 || s != 2 {
		t.Errorf("GetBoatSpeed(60, 39.1, hull) = (%f, %d); want (14.644201, 2)", ws, s)
	}

	ws, s, _ = z.GetBoatSpeed(60, 39.1, Boat{Foil: true})
	if math.Round(ws*1000000) != 14600400 || s != 2 {
		t.Errorf("GetBoatSpeed(60, 39.1, hull) = (%f, %d); want (14.600400, 2)", ws, s)
	}

	ws, s, _ = z.GetBoatSpeed(43, 38.6, Boat{})
	if math.Round(ws*1000000) != 14600400 || s != 2 {
		t.Errorf("GetBoatSpeed(60, 39.1, hull) = (%f, %d); want (14.600400, 2)", ws, s)
	}
}

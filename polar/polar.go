package polar

import (
	"fmt"
	"io/ioutil"
	"math"
)

type Polar struct {
	options    Options
	polar_data [8][]byte
}

type Options struct {
	Race string
	Sail int
}

type Boat struct {
	Foil bool
	Hull bool
}

func Init(o Options) Polar {
	var p [8][]byte
	i := 0
	for i < 8 {

		b, err := ioutil.ReadFile("polars/" + o.Race + "_" + fmt.Sprintf("%d", i))
		if err != nil {
			fmt.Print(err)
		}
		p[i] = b
		i++
	}
	z := Polar{}
	z.options = o
	z.polar_data = p
	return z
}

func (z Polar) GetBoatSpeed(twa float64, ws float64, boat Boat) (float64, byte) {
	return z.GetBoatSpeed2(twa, ws, boat, z.options.Sail)
}

func (z Polar) GetOptimBoatSpeed(twa float64, ws float64, boat Boat, s byte, winchMalus float64) (float64, byte) {
	o := 0
	if s == 2 || s == 3 {
		o = 1
	}
	if s == 4 {
		o = 2
	}
	if s == 5 || s == 6 {
		o = 4
	}
	bs1, s1 := z.GetBoatSpeed2(twa, ws, boat, o)
	bs2, s2 := z.GetBoatSpeed(twa, ws, boat)

	d1 := bs1 * 1.852 * 1.0 * 1000.0
	d2 := bs2 * 1.852 * 1.0 * 1000.0
	if int(s1) != s {
		d1 = bs1 * 1.852 * (1.0*60.0 - winchMalus/2) / 60 * 1000.0
	}
	if int(s2) != s {
		d2 = bs2 * 1.852 * (1.0*60.0 - winchMalus/2) / 60 * 1000.0
	}

	if d1 > d2 {
		return bs1, s1
	} else {
		return bs2, s2
	}
}

func (z Polar) GetBoatSpeed2(twa float64, ws float64, boat Boat, s int) (float64, byte) {
	p := z.polar_data[s]
	t := twa
	if t < 0 {
		t = -1 * t
	}
	if t > 180 {
		t = 360 - t
	}

	if ws >= 129 {
		ws = 129
	}

	var offset int

	ws1 := int(math.Floor(ws))
	frac := ws - float64(ws1)
	offset = 4 * (130*int(math.Round(t)) + ws1)
	bs := (float64(p[offset+1])*256 + float64(p[offset+2])) / 100
	bs1 := (float64(p[offset+5])*256 + float64(p[offset+6])) / 100
	sail := p[offset]
	retbs := bs + frac*(bs1-bs)
	if boat.Foil {
		retbs *= foil(t, ws/1.852)
	}
	if boat.Hull {
		retbs *= 1.003
	}
	return retbs, sail
}

func foil(twa, ws float64) float64 {
	ct := 0.0
	cv := 0.0
	if twa <= 70 {
		return 1.0
	} else if twa < 80 {
		ct = float64(twa-70) / 10.0
	} else if twa < 160 {
		ct = 1
	} else if twa < 170 {
		ct = float64(170-twa) / 10.0
	} else {
		return 1.0
	}
	if ws <= 11 {
		return 1.0
	} else if ws < 16 {
		cv = (ws - 11) / 5
	} else if ws < 35 {
		cv = 1
	} else if ws < 40 {
		cv = (40 - ws) / 5
	} else {
		return 1.0
	}
	return 1.0 + 0.04*ct*cv
}

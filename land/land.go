package land

import (
	"io/ioutil"
	"math"

	log "github.com/sirupsen/logrus"
)

// Land contains slice containing 0 if sea and 1 if land
type Land struct {
	lat0 float64
	latN float64
	lon0 float64
	lonN float64
	step float64
	data []byte
}

// InitLand load lands file
func InitLand() (*Land, error) {
	file := "land/output"

	b, err := ioutil.ReadFile(file)
	if err != nil {
		log.Errorf("Error reading file '%s'", file)
		return nil, err
	}
	return &Land{
		lat0: -90.0,
		latN: 90.0,
		lon0: -180.0,
		lonN: 180.00 - 360.0/43200.0,
		step: 360.0 / 43200.0,
		data: b}, nil
}

// IsLand check if location is land or sea
func (l Land) IsLand(lat float64, lon float64) bool {
	i := int(math.Round(lat / l.step))
	j := int(math.Round(lon / l.step))

	i0 := int(l.lat0 / l.step)
	j0 := int(l.lon0 / l.step)
	jN := int(l.lonN / l.step)

	di := i - i0
	dj := j - j0
	nj := jN - j0 + 1

	p := di*nj + dj

	pB := p / 8
	pb := uint(p % 8)

	return ((l.data[pB] >> (7 - pb)) & 0x01) == 0x01
}

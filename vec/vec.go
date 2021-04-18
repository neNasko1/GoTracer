package vec

import (
	"math"
	"math/rand"
)

type Vector3d struct {
	X, Y, Z float64
}

func RandomInUnit() Vector3d {
	for {
		ret := Sub(Mult(Vector3d{rand.Float64(), rand.Float64(), rand.Float64()}, 2.0), Vector3d{1, 1, 1})
		if Length(ret) < 1.0 {
			return ret
		}
	}
}
func FixCol(col *Vector3d, n int) {
	*col = Mult(*col, 1.0 / float64(n))
	col.X = math.Sqrt(col.X); col.Y = math.Sqrt(col.Y); col.Z = math.Sqrt(col.Z)
}
func Add(a, b Vector3d) Vector3d {
	return Vector3d{a.X + b.X, a.Y + b.Y, a.Z + b.Z}
}
func Sub(a, b Vector3d) Vector3d {
	return Vector3d{a.X - b.X, a.Y - b.Y, a.Z - b.Z}
}
func Mult(a Vector3d, s float64) Vector3d {
	return Vector3d{a.X * s, a.Y * s, a.Z * s}
}
func MultVec(a, b Vector3d) Vector3d {
	return Vector3d{a.X * b.X, a.Y * b.Y, a.Z * b.Z}
}
func Dot(a, b Vector3d) float64 {
	return a.X * b.X + a.Y * b.Y + a.Z * b.Z
}
func Cross(a, b Vector3d) Vector3d {
	return Vector3d{a.Y * b.Z - a.Z * b.Y, -(a.X * b.Z - a.Z * b.X), a.X * b.Y - a.Y * b.X}
}
func SquaredLength(a Vector3d) float64 {
	return a.X * a.X + a.Y * a.Y + a.Z * a.Z
}
func Length(a Vector3d) float64 {
	return math.Sqrt(SquaredLength(a))
}
func Norm(a Vector3d) Vector3d {
	return Mult(a, float64(1) / Length(a))
}
func Reflect(a, n Vector3d) Vector3d {
	return Sub(a, Mult(n, 2 * Dot(a, n)))
}
func Refract(a, n Vector3d, nRatio float64) (bool, Vector3d) {
	unit := Norm(a)
	dot := Dot(unit, n)
	D := 1.0 - nRatio * nRatio * (1.0 - dot * dot)
	if D > 0 {
		return true, Sub(Mult(Sub(unit, Mult(n, dot)), nRatio), Mult(n, math.Sqrt(D)))
	} else {
		return false, Vector3d{0, 0, 0}
	}
}
type Ray struct {
	Orig, Dir Vector3d
}
func PointOn(r Ray, t float64) Vector3d {
	return Add(r.Orig, Mult(r.Dir, t))
}

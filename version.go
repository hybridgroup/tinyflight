package tinyflight

const version = "0.4.0"

// Version returns the tinyflight version.
func Version() string {
	return version
}

// VersionNumbers returns the major, minor and patch numbers of the version.
func VersionNumbers() [3]uint8 {
	var v [3]uint8
	i := 0
	for _, c := range version {
		switch {
		case c == '.':
			i++
			if i == len(v) {
				return v
			}
		case c >= '0' && c <= '9':
			v[i] = v[i]*10 + uint8(c-'0')
		default:
			return v
		}
	}
	return v
}

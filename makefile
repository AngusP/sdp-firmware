all:
	verify, upload

verify:
	arduino --verify ./ --verbose-build

upload:
	arduino --upload ./ --verbose

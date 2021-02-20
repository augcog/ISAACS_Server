all: tests

tests:
	@echo "Running drone tests..."
	python3 -m unittest -v test

working:
	@echo "Running TestVRConnection..."
	python3 -m unittest -v test.TestVRConnection
	@echo "Running TestMavrosCreation..."
	python3 -m unittest -v test.TestMavrosCreation
	@echo "Running TestDjimatriceCreation..."
	python3 -m unittest -v test.TestDjimatriceCreation

file_finder = find . -type f $(1) -not \( -path '*/venv/*' -o -path '*run_roi_prediction*' \)

CMAKE_FILES = $(call file_finder,-name "*.cmake" -o -name "CMakeLists.txt")
PY_FILES = $(call file_finder,-name "*.py")
SH_FILES = $(call file_finder,-name "*.sh")

check: check_format check_sh_format pylint shellcheck

format:
	$(PY_FILES) | xargs black
	$(CMAKE_FILES) | xargs cmake-format -i
	shfmt -l -w .

check_format:
	$(PY_FILES) | xargs black --diff --check
	$(CMAKE_FILES) | xargs cmake-format --check

pylint:
	$(PY_FILES) | xargs pylint --rcfile=.pylintrc

check_sh_format:
	shfmt -d .

shellcheck:
	$(SH_FILES) | xargs shellcheck

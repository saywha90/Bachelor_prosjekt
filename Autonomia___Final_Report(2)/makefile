OUT = sortify_documentation

.PHONY : default_target
default_target : all

.PHONY : clean
clean :
	@echo "Deleting $(OUT).pdf symlink"
	@$(RM) $(OUT).pdf
	@echo "Deleting build directory"
	@$(RM) -r build

.PHONY : all
all : build/$(OUT)
	@echo "Creating symlink: $(OUT).pdf -> $<"
	@$(RM) $(OUT).pdf
	@ln -s build/$(OUT).pdf $(OUT).pdf

build/$(OUT) : dir
	xelatex -shell-escape -output-directory=build -jobname=$(OUT) ./main.tex
	bibtex build/$(OUT).aux
	cd build && makeglossaries $(OUT)
	xelatex -shell-escape -output-directory=build -jobname=$(OUT) ./main.tex
	xelatex -shell-escape -output-directory=build -jobname=$(OUT) ./main.tex

.PHONY : dir
dir :
	@echo "Creating build directory"
	@mkdir -p build

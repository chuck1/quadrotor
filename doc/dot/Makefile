
dot_files = pos.dot

pdf_files = pos.pdf

all: $(pdf_files)

%.pdf: %.dot
	dot $^ -Tpdf > $@


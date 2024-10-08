EXECUTE = SimpleNavigator

GRAPH_LIB = s21_graph.a
GRAPH_ALGORITHMS_LIB = s21_graph_algorithms.a

DIR_BUILD = build
DIR_INSTALL = out

DIR_TEST = unit_tests
TEST_BUILD = unit_tests/build
TEST_EXECUTE = tests

DIR_REPORT = report
HTML_REPORT = REPORT.html
EXCLUDE_FILES = "unit_tests/*" "Navigator/Components/*"	"Navigator/Graph/s21_graph.h"	
EXCLUDE_REPORT = $(foreach file, $(EXCLUDE_FILES), --exclude $(file)) 

DISTRIBUTION = $(EXECUTE).tar


all: install

$(GRAPH_LIB):
	@mkdir -p ./${DIR_BUILD}
	cd $(DIR_BUILD) && cmake -DBUILD_GRAPH_LIBRARY=ON .. && make && mv $(GRAPH_LIB) ..
	@make rm_build

$(GRAPH_ALGORITHMS_LIB):
	@mkdir -p ./${DIR_BUILD}
	cd $(DIR_BUILD) && cmake -DBUILD_GRAPH_ALGORITHMS_LIBRARY=ON .. && make && mv $(GRAPH_ALGORITHMS_LIB) ..
	@make rm_build

install:
	@mkdir -p ./${DIR_INSTALL}
	@mkdir -p ./${DIR_BUILD}
	cd $(DIR_BUILD) && cmake -DBUILD_NAVIGATOR_EXE=ON .. && make && mv $(EXECUTE) ../${DIR_INSTALL}/
	@make rm_build
	@make run

run:
	./out/${EXECUTE}

uninstall:
	@rm -rf $(DIR_INSTALL)

tests:
	@mkdir -p ./$(TEST_BUILD)
	cd $(TEST_BUILD) && cmake .. && make && mv $(TEST_EXECUTE) ..
	@cd $(DIR_TEST) && ./${TEST_EXECUTE}

gcov_report: tests
	@mkdir -p $(DIR_REPORT)
	@find ./$(TEST_BUILD)/CMakeFiles \( -name "*.gcno" -o -name "*.gcda"  \) -exec mv {} $(DIR_REPORT)/ \;
	~/.local/bin/gcovr $(DIR_REPORT)/ $(EXCLUDE_REPORT) --html --html-details -o $(HTML_REPORT)
	@make move_report

move_report:
	@mv *.html *.css $(DIR_REPORT)

peer: gcov_report
	@open $(DIR_REPORT)/$(HTML_REPORT)

dvi:
	@doxygen ./doc/Doxyfile
#	@google-chrome doc/html/index.html

dist:
	@if [ -d "$(DIR_INSTALL)" ]; then \
		(cd $(DIR_INSTALL) && tar -cf $(DISTRIBUTION) . && mv $(DISTRIBUTION) ../); \
	else \
		echo "Directory $(DIR_INSTALL) does not exist."; \
	fi

clean:
	@make rm_build
	@rm -rf ./$(DIR_TEST)/$(TEST_EXECUTE)
	@rm -rf ./$(DIR_TEST)/test_files/*.dot
	@rm -rf ./$(DIR_REPORT)
	@rm -rf *.a

rm_build:
	@rm -rf $(DIR_BUILD)
	@rm -rf $(TEST_BUILD)

rebuild: clean uninstall install
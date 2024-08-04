EXECUTE = SimpleNavigator
DIR_BUILD = build
DIR_INSTALL = out

all: install

install: clean uninstall
	@mkdir -p ./out
	@mkdir -p ./build
	cd $(DIR_BUILD) && cmake .. && make && mv $(EXECUTE) ../${DIR_INSTALL}/
	@make rm_build
	@make run

run:
	@cd $(DIR_INSTALL) && ./${EXECUTE}

uninstall:
	@rm -rf $(DIR_INSTALL)

clean: rm_build
#	@rm -rf ./$(DIR_TEST)/$(TEST_EXECUTE)
#	@rm -rf ./$(DIR_TEST)/generatedMazes
#	@rm -rf ./$(DIR_REPORT)

rm_build:
	@rm -rf $(DIR_BUILD)
#	@rm -rf $(TEST_BUILD)

rebuild: clean install
.PHONY: help test lint compile dry-run run clean

PY ?= python3
ROOT := $(shell pwd)

help:
	@echo "make test       跑纯逻辑单测（不需要 ROS）"
	@echo "make compile    语法编译检查"
	@echo "make lint       静态检查（需要 ruff，可选）"
	@echo "make dry-run    打印任务配置与航线（不需要 ROS）"
	@echo "make run        跑完整任务（需要仿真已启动）"
	@echo "make clean      清理 __pycache__"

test:
	PYTHONPATH=src $(PY) -m unittest discover -s tests -v

compile:
	$(PY) -m compileall -q src scripts tests

lint:
	@command -v ruff >/dev/null 2>&1 && ruff check src tests || echo "ruff 未安装，跳过"

dry-run:
	PYTHONPATH=src $(PY) -m autofly.cli --dry-run

run:
	PYTHONPATH=src $(PY) -m autofly.cli

clean:
	find . -name __pycache__ -type d -prune -exec rm -rf {} + 2>/dev/null || true
	find . -name "*.pyc" -delete 2>/dev/null || true
	@echo "cleaned"

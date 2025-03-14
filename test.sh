STM32PY_PATH="$1/build/stm32py"

if [ ! -d "$STM32PY_PATH" ]; then
    python3 -m venv $STM32PY_PATH
fi

source $STM32PY_PATH/bin/activate

pip show pyserial > /dev/null 2>&1
if [ $? -ne 0 ]; then
    pip install pyserial
fi

python3 $1/TEST/scripts/run_test.py $2 && deactivate && rm -rf $STM32PY_PATH  && python3 $1/TEST/scripts/report.py

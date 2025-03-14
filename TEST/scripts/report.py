import os
import re
import sys
from collections import defaultdict
from datetime import datetime

# remove garbage #############################################
def clean_details(details):
    details = re.sub(r'\.[A-Za-z]\.+', '', details) 
    details = re.sub(r'[.\^]+', '', details)  
    details = re.sub(r'\s+', ' ', details)   
    return details.strip()


# Get MCU model from CMakeLists.txt ##########################
def extract_mcu_model(cmake_file):
    if not os.path.isfile(cmake_file):
        print(f"CMakeLists.txt not found: {cmake_file}")
        return "Unknown MCU"

    
    with open(cmake_file, 'r') as f:
        cmake_content = f.read()

    mcu_model_match = re.search(r'set\s*\(\s*MCU_MODEL\s+([^\s]+)\s*\)', cmake_content, re.IGNORECASE)
    if mcu_model_match:
        return mcu_model_match.group(1)
    else:
        return "Unknown MCU"

# get project name from CMakeLists.txt######################
def extract_project_name(cmake_file):
    if not os.path.isfile(cmake_file):
        print(f"CMakeLists.txt not found: {cmake_file}")
        return "Unknown Project"

    # get project name
    with open(cmake_file, 'r') as f:
        cmake_content = f.read()

    project_name_match = re.search(r'project\s*\(\s*([^\s]+)\s*\)', cmake_content, re.IGNORECASE)
    if project_name_match:
        return project_name_match.group(1)
    else:
        return "Unknown Project"

# decide on test environment according to compiler name ####
def extract_test_environment(cmake_file):
    if not os.path.isfile(cmake_file):
        print(f"CMakeLists.txt not found: {cmake_file}")
        return "Unknown Environment"

    with open(cmake_file, 'r') as f:
        cmake_content = f.read()

    # check for any mention of ARM in the cmake file
    if "arm-none-eabi" in cmake_content.lower():
        return "On-target"
    else:
        return "On-host"
    
# generate html file #########################################
def generate_html_report(log_file, output_html_file, cmake_file):
    # check if the log file exists
    if not os.path.isfile(log_file):
        print(f"Log file not found: {log_file}")
        return

    with open(log_file, 'r') as f:
        log_content = f.read()

    # remove garbage
    clean_log_content = re.sub(r'[^A-Za-z0-9\s<>\[\]\(\),._:=-]', '', log_content)

    # capture all test failure outputs
    test_failure_format = re.compile(
        r"(?P<file>.+?):(?P<line>\d+): error: Failure in TEST\((?P<group>.+?), (?P<test>.+?)\)\s*(?P<details>.+?)(?=\n\S+?:\d+:|Errors|$)", 
        re.DOTALL
    )

    # capture test summary for both failed and successful test runs
    test_summary_format_errors = re.compile(
        r"Errors \((?P<failures>\d+) failures, (?P<total>\d+) tests, (?P<ran>\d+) ran, \d+ checks.*\)"
    )
    
    test_summary_format_ok = re.compile(
        r"OK \((?P<total>\d+) tests, (?P<ran>\d+) ran, \d+ checks, \d+ ignored, \d+ filtered out, \d+ ms\)"
    )

    # find all failed tests
    failures_by_group = defaultdict(list)
    failures = test_failure_format.findall(clean_log_content)

    # try parsing summary using the two patterns
    summary_match_errors = test_summary_format_errors.search(clean_log_content)
    summary_match_ok = test_summary_format_ok.search(clean_log_content)

    if summary_match_errors:
        failures_count = int(summary_match_errors.group("failures"))
        total_tests = int(summary_match_errors.group("total"))
        passed_tests = total_tests - failures_count
    elif summary_match_ok:
        failures_count = 0
        total_tests = int(summary_match_ok.group("total"))
        passed_tests = total_tests
    else:
        failures_count = total_tests = passed_tests = 0

    # group failed tests by their group name
    for failure in failures:
        group = failure[2].strip()  # Test group name
        test_name = failure[3].strip()  # Test name
        details = clean_details(failure[4].strip())  # clean failure details
        failures_by_group[group].append((test_name, details))

    # get current time
    current_time = datetime.now().strftime("%d-%m-%Y %H:%M:%S")

    # get MCU model, project name, and test environment from CMakeLists.txt
    mcu_model = extract_mcu_model(cmake_file)
    project_name = extract_project_name(cmake_file)
    test_environment = extract_test_environment(cmake_file)

    # create HTML document template
    html_content = f"""
    <!DOCTYPE html>
    <html>
    <head>
        <title>Unit Test Report</title>
        <link rel="preconnect" href="https://fonts.googleapis.com">
        <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
        <link href="https://fonts.googleapis.com/css2?family=Inter:ital,opsz,wght@0,14..32,100..900;1,14..32,100..900&display=swap" rel="stylesheet">
        <style>
            body {{ 
                font-family: "Inter", sans-serif;
                background-color: #181818; 
                color: #F0F0F0; 
                margin-left: 5.5em;
                padding: 20px;
            }}
            h1, h2, h3 {{
                font-weight: bold;
                color: #11D8E8; 
            }}
            h1 {{ 
                border-bottom: 2px solid #11D8E8; 
                padding-bottom: 10px;
            }}
            .label {{
                font-weight: bold;
                color: #BCBCAC;
            }}
            .value {{
                font-weight: normal;
                color: #FFFFFF;
            }}
            table {{
                width: 100%;
                border-collapse: collapse;
                margin: 20px 0;
                font-size: 1em;
                box-shadow: 0 0 10px rgba(0, 0, 0, 0.1);
            }}
            table th, table td {{
                padding: 12px 15px;
                border: 1px solid #ddd;
                text-align: left;
            }}
            table th {{
                background-color: #272727;
                color: #BCBCAC;
            }}
            table tr:nth-of-type(even) {{
                background-color: #272727;
            }}
            .status-pass {{ 
                color: green; 
                font-weight: bold;
            }}
            .status-fail {{ 
                color: red; 
                font-weight: bold;
            }}
            .test-details {{ 
                font-size: 0.9em; 
                color: #ACAC60; 
            }}
        </style>
    </head>
    <body>
        <h1>Unit Test Report</h1>

        <h2>Project Information</h2>
        <p><span class="label">MCU Model:</span> <span class="value">{mcu_model}</span></p>
        <p><span class="label">Project Name:</span> <span class="value">{project_name}</span></p>
        <p><span class="label">Test Environment:</span> <span class="value">{test_environment}</span></p>
        <p><span class="label">Report Generated:</span> <span class="value">{current_time}</span></p>

        <h2>Test Summary</h2>
        <p><span class="status-pass">PASSED: {passed_tests}</span></p>
        <p><span class="status-fail">FAILED: {failures_count}</span></p>
        <p>TOTAL: {total_tests}</p>

        <h2>Test Results</h2>
        <table>
            <thead>
                <tr>
                    <th>Test Group</th>
                    <th>Test Name</th>
                    <th>Status</th>
                    <th>Details</th>
                </tr>
            </thead>
            <tbody>
    """

    # go trough each group and its tests
    if failures_by_group:
        for group, tests in failures_by_group.items():
            for test_name, details in tests:
                html_content += f"""
                <tr>
                    <td>{group}</td>
                    <td>{test_name}</td>
                    <td><span class="status-fail">Failed</span></td>
                    <td class="test-details">{details}</td>
                </tr>
                """
    else:
        html_content += """
        <tr>
            <td colspan="4" class="status-pass">All tests passed!</td>
        </tr>
        """

    # close html doc
    html_content += """
            </tbody>
        </table>
    </body>
    </html>
    """

    # write the html doc to the actual file
    with open(output_html_file, 'w') as html_file:
        html_file.write(html_content)

# locate script folder 
script_dir = os.path.dirname(os.path.abspath(__file__))

# find build folder from here
build_folder = os.path.join(script_dir,'..','..','build')
log_file = os.path.join(build_folder, "uart_output.log")
output_html_file = os.path.join(build_folder, "UNIT_TEST_REPORT.html")
cmake_file = os.path.join(script_dir, '..', '..', 'CMakeLists.txt')

# generate report with data from log file
generate_html_report(log_file, output_html_file, cmake_file)

# search OK
def check_log_for_ok(uart_output):
    with open(log_file, 'r') as f:
        for line in f:
            if "OK" in line:
                return True
    return False

# return 0 on OK
if check_log_for_ok(log_file):
    sys.exit(0)
else:
    sys.exit(1)

# print(f"HTML report generated in /build folder: {output_html_file}")

##############################################################

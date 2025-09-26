def bug_func(val) :
    a = str_val["string"]



str_val = '{"string": {"dict": 9}}'
a = str_val["string"]
bug_func(str_val)

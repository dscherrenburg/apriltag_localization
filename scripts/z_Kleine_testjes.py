lst = [[2], [3, 2], [4]]

def index_in_list(a_list, index):
    return index < len(a_list)

if index_in_list is None:
    print('Bestaat niet')

for l in lst:
    if len(l) > len(lst[-1]):
        del l[-1]

print(lst)


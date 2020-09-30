import xlrd
wb = xlrd.open_workbook('RA IK DATA.xlsx')
geometry = wb.sheet_by_index(0)
if __name__ == "__main__":
    print(geometry.cell(7,3))
    




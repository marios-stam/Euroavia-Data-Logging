import xlsxwriter
import plot as graphs
import process_txt as txt
TIME_INTERVAL=0.5

logs=txt.Logs

# Create a workbook and add a worksheet.
workbook = xlsxwriter.Workbook('Rocket_data.xlsx')
worksheet = workbook.add_worksheet()

#Make custom formats to use 
title = workbook.add_format({'bold': True,'align':'center'})

titles=['Time','Altitude','Temperature','Acceleration-x','Acceleration-y','Acceleration-z','Orientation-x','Orientation-y','Orientation-z','Pressure']
for col,i in enumerate(titles):
    worksheet.set_column(col,col,len(i))#make column fit the title
    worksheet.write(0,col,i,title)

for row,i in enumerate(logs):
    row=row+1#first line is for titles
    worksheet.write(row, 0,row*TIME_INTERVAL )
    worksheet.write(row, 1,i.alt )
    worksheet.write(row, 2,i.temp )
    worksheet.write(row, 3,i.acceleration[0] )
    worksheet.write(row, 4,i.acceleration[1] )
    worksheet.write(row, 5,i.acceleration[2] )
    worksheet.write(row, 6,i.orientation[0] )
    worksheet.write(row, 7,i.orientation[1] )
    worksheet.write(row, 8,i.orientation[2] )
    worksheet.write(row, 9,i.pressure )


workbook.close()

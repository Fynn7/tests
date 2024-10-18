function main(workbook: ExcelScript.Workbook) {
    let selectedCell = workbook.getActiveCell();
    let selectedSheet = workbook.getActiveWorksheet();

    selectedCell.getFormat().getFill().setColor("yellow");

}

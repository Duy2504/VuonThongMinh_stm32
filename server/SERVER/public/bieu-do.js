
const chart = Highcharts.chart("container", {
  chart: {
    zoomType: 'xy'
  },
  title: {
    text: 'Average hourly Temperature, Humidity and brightness in PTIT',
    align: 'left'
  },
  subtitle: {
    text: 'Source: ' +
      '<a href="https://www.yr.no/nb/historikk/graf/5-97251/Norge/Troms%20og%20Finnmark/Karasjok/Karasjok?q=2021"' +
      'target="_blank">YR</a>'
  },
  xAxis: [{
    categories: [],
    crosshair: true
  }],
  yAxis: [{ // Primary yAxis
    labels: {
      format: '{value}°C',
      style: {
        color: Highcharts.getOptions().colors[1]
      }
    },
    title: {
      text: 'Temperature',
      style: {
        color: Highcharts.getOptions().colors[1]
      }
    }
  }, { // Secondary yAxis
    title: {
      text: 'Humidity + Brightness',
      style: {
        color: Highcharts.getOptions().colors[0]
      }
    },
    labels: {
      format: '{value} %',
      style: {
        color: Highcharts.getOptions().colors[0]
      }
    },
    opposite: true
  }],
  tooltip: {
    shared: true
  },
  legend: {
    align: 'left',
    x: 80,
    verticalAlign: 'top',
    y: 80,
    floating: true,
    backgroundColor:
      Highcharts.defaultOptions.legend.backgroundColor || // theme
      'rgba(255,255,255,0.25)'
  },

  series: [
    {
      name: "Temperature",
      type: 'column',
      
      // data: [
      //   43, 48, 65, 81, 11, 14, 17, 16, 15,
      //   16, 15,
      // ],
      // data: arrayN
      tooltip: {
        valueSuffix: '°C'
      }
    },
    {
      name: "Humidity",
      type: 'spline',
      yAxis: 1,
      // data: [24, 37, 29, 29, 32, 30,
      //   38, 36, 33, 34, 31]
      // data: arrayN,
      tooltip: {
        valueSuffix: ' %'
      }
    },
    {
      name: "Brightness",
      type: 'spline',
      // data: [
      //   11, 30, 16, 19, 20, 24, 32, 30, 29, 29,
      //   25,
      // ],
      tooltip: {
        valueSuffix: ' %'
      }
    },
    {
      name: "Dust",
      type: 'spline',
      // data: [
      //   11, 30, 16, 19, 20, 24, 32, 30, 29, 29,
      //   25,
      // ],
      tooltip: {
        valueSuffix: ' %'
      }
    },
  ],

  responsive: {
    rules: [
      {
        condition: {
          maxWidth: 500,
        },
        chartOptions: {
          legend: {
            layout: "horizontal",
            align: "center",
            verticalAlign: "bottom",
          },
        },
      },
    ],
  },
});

export function redrawChart(dataT, dataH, dataL,dataD) {
  chart.series[0].setData(dataT);
  chart.series[1].setData(dataH);
  chart.series[2].setData(dataL);
  chart.series[3].setData(dataD);
  // console.log();
}

dotnet publish -r linux-arm /p:ShowLinkerSizeComparison=true 
pushd F:\git\iot\artifacts\bin\System.Device.Gpio.Tests\Debug\netcoreapp2.1\linux-arm\publish
scp -r .\* pi@joespi:/home/pi/System.Device.Gpio.Tests
popd
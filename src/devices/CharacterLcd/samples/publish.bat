dotnet publish -r linux-arm
pushd .\bin\Debug\netcoreapp2.1\linux-arm\publish
scp -r .\* pi@joespi:/home/pi/lcdDemo
popd
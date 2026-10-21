# Payload component sources

* `interfaces/`: abstract execution contracts, one class-named header per type.
* `implementations/<role>/`: concrete models; every class has a matching
  `<ClassName>.h` declaration and `<ClassName>.cpp` method definitions.
* `support/`: abstract helpers for native asset binding and release lifecycle.
* `registration/`: built-in registration, catalogue assembly and construction.
* `ProviderContextV1.h`, `ProviderTableV1.h`, `ProviderRegistryV1.h/.cpp`:
  factory inputs, typed tables and per-World registration catalogue.
* `PlantProvidersV1.h/.cpp`: selected native command-stage composition.
* `Profile.h/.cpp`, `Bindings.h/.cpp`: configuration and immutable card binding.
* `Providers.h`: compatibility umbrella; prefer a specific contract include.

See the [class index and extension contract](../../../../docs/sdk/payload-native-providers.md).
Keep concrete algorithms out of role interfaces and registration files.
Different supported parameters can share a concrete model; a different
algorithm requires another concrete class and its own regression evidence.

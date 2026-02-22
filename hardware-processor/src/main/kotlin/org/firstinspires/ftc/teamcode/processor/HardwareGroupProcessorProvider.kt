package org.firstinspires.ftc.teamcode.processor

import com.google.devtools.ksp.processing.SymbolProcessor
import com.google.devtools.ksp.processing.SymbolProcessorEnvironment
import com.google.devtools.ksp.processing.SymbolProcessorProvider

class HardwareGroupProcessorProvider : SymbolProcessorProvider {
    override fun create(environment: SymbolProcessorEnvironment): SymbolProcessor =
        HardwareGroupProcessor(
            codeGenerator = environment.codeGenerator,
            logger        = environment.logger,
        )
}
